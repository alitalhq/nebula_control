import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import CameraInfo
from nebula_interfaces.msg import BalloonArray, RectangleArray, GimbalFeedback, GimbalMode
from nebula_interfaces.srv import SetMode

import serial
import struct
import threading
import math
import time

class OperationManagerNode(Node):

    # Define operational modes
    MODE_SAFE = 0
    MODE_SEARCH = 1
    MODE_LASER = 2

    def __init__(self):
        super().__init__('operation_manager_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('centering_threshold', 0.01)
        self.declare_parameter('lock_duration', 3.0)

        # Parametre değerlerini oku
        self.serial_port_name = self.get_parameter('serial_port').value
        self.baud_rate_val = self.get_parameter('baudrate').value
        self.threshold = self.get_parameter('centering_threshold').value
        self.lock_duration = self.get_parameter('lock_duration').value

        self.add_on_set_parameters_callback(self.parameter_callback)
    
        feedback_qos = QoSProfile(depth=1)
        feedback_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        feedback_qos.history = HistoryPolicy.KEEP_LAST

        cam_qos = QoSProfile(depth=1)
        cam_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cam_qos.history = HistoryPolicy.KEEP_LAST

        target_qos = QoSProfile(depth=1)
        target_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = QoSReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # -------------- Serial ---------------
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate_val,
                timeout=0.01
            )
            self.get_logger().info('Serial connected')
        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f'Serial error: {e}')

        if self.serial_port:
            self.telemetry_thread = threading.Thread(
                target=self.telemetry_reader_loop,
                daemon=True
            )
            self.telemetry_thread.start()
            self.get_logger().info('Telemetry reader thread started')

        self.command_timer = self.create_timer(
            0.1,  # 10 Hz
            self.command_timer_callback
        )
        
        self.last_target_time = time.time()
        self.target_timeout = 0.5  # 0.5 seconds without target → send home command
        self.last_commanded_pan = 0.0
        self.last_commanded_tilt = 0.0
        # Subscriptions
        self.balloons_sub = self.create_subscription(BalloonArray, '/vision/balloons', self.balloons_callback, target_qos)
        self.rectangles_sub = self.create_subscription(RectangleArray, '/vision/rectangles', self.rectangles_callback, target_qos)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, cam_qos)

        # Publishers
        self.gimbal_mode_pub = self.create_publisher(GimbalMode, '/gimbal/mode', mode_qos)
        self.feedback_pub = self.create_publisher(GimbalFeedback, '/gimbal/feedback', feedback_qos)

        """
        # Clients
        self.laser_fire_client = self.create_client(FireCommand, '/laser/fire')
        while not self.laser_fire_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('laser_fire service not available, waiting again...')
        """

        # Services
        self.set_mode_service = self.create_service(SetMode, '/gimbal/set_mode', self.set_mode_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_w = None
        self.img_h = None

        self.current_mode = self.MODE_LASER #TESTLERDEN SONRA MODE_SAFE YAPMAYI UNUTMA
        self.gimbal_mode_pub.publish(GimbalMode(mode=self.current_mode))
        self.get_logger().info(f'Operation Manager Node has been started in {self.current_mode} mode.')

        # Internal state variables
        self.last_gimbal_feedback = None
        self.last_balloons = None
        self.last_rectangles = None

        self.lock_until = 0
        self.is_waiting = False

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.img_w = msg.width
        self.img_h = msg.height

    def set_mode_callback(self, request, response):
        if request.mode in [self.MODE_SAFE, self.MODE_SEARCH, self.MODE_LASER]:
            self.current_mode = request.mode
            self.gimbal_mode_pub.publish(GimbalMode(mode=self.current_mode))
            response.success = True
            response.message = f"Mode set to {self.current_mode}"
            self.get_logger().info(f'Mode changed to: {self.current_mode}')
        else:
            response.success = False
            response.message = f"Invalid mode: {request.mode}"
            self.get_logger().warn(f'Invalid mode requested: {request.mode}')
        return response

    def balloons_callback(self, msg):
        current_time = time.time()

        if current_time < self.lock_until:
            if not self.is_waiting:
                self.get_logger().info("System locked for 3 seconds, waiting...")
                self.is_waiting = True
            return
        self.is_waiting = False

        if self.current_mode != self.MODE_LASER or not msg.balloons or self.fx is None:
            return
        
        self.last_target_time = time.time()
        
        target = min(msg.balloons, key=lambda b: ((b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2))

        absolute_pan, absolute_tilt = self.norm_to_angle(target.u_norm, target.v_norm)
        
        pan_delta = absolute_pan - self.last_commanded_pan
        tilt_delta = absolute_tilt - self.last_commanded_tilt
        
        # Update last commanded angles
        self.last_commanded_pan = absolute_pan
        self.last_commanded_tilt = absolute_tilt

        is_centered = abs(target.u_norm - 0.5) < self.threshold and abs(target.v_norm - 0.5) < self.threshold

        fire_signal = False
        if is_centered:
            fire_signal = True
            self.lock_until = current_time + self.lock_duration
            self.get_logger().info(f"Laser Armed! Target Centered. Locking for {self.lock_duration}s...")

        self.send_to_mcu(pan_delta, tilt_delta, True, fire_signal)


        fb = GimbalFeedback()
        fb.pan_deg, fb.tilt_deg = pan_delta, tilt_delta
        self.feedback_pub.publish(fb)

    def rectangles_callback(self, msg):
        self.last_rectangles = msg


    def norm_to_angle(self, u_norm, v_norm):

        u = u_norm * self.img_w
        v = v_norm * self.img_h

        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        pan = math.degrees(math.atan(x))
        tilt = -math.degrees(math.atan(y))

        return pan, tilt
    
    def send_to_mcu(self, pan_delta, tilt_delta, laser_enable, laser_fire):
        """
        Binary packet structure (22 bytes total):
        - [0-1]   Header: 0xAA 0xFF
        - [2-5]   pan_delta (float32)
        - [6-9]   tilt_delta (float32)
        - [10-13] feedforward_vel_pan (float32) - set to 0 for now
        - [14-17] feedforward_vel_tilt (float32) - set to 0 for now
        - [18]    laser_enable (uint8)
        - [19]    laser_fire (uint8)
        - [20-21] CRC16
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return
    
        try:
            # Build packet (without CRC first)
            packet_data = struct.pack(
                '<BBffffBB',
                0xAA,                    # Header 1
                0xFF,                    # Header 2
                float(pan_delta),        # Pan delta (degrees)
                float(tilt_delta),       # Tilt delta (degrees)
                0.0,                     # Feedforward vel pan (not used yet)
                0.0,                     # Feedforward vel tilt (not used yet)
                int(laser_enable),       # Laser enable flag
                int(laser_fire)          # Laser fire flag
            )
            
            # Calculate CRC16
            crc = self.calculate_crc16(packet_data)
            
            # Append CRC (little-endian uint16)
            packet = packet_data + struct.pack('<H', crc)
            
            # Send packet
            self.serial_port.write(packet)
            
            # Optional: Log for debugging
            # self.get_logger().debug(f'Sent: pan={pan_delta:.2f}, tilt={tilt_delta:.2f}, fire={laser_fire}')
            
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


    
    def telemetry_reader_loop(self):
        """Background thread to continuously read telemetry from MCU"""
        buffer = bytearray()
        TELEMETRY_SIZE = 32  # From SerialProtocol: 32 bytes
        
        while rclpy.ok():
            try:
                if self.serial_port and self.serial_port.is_open:
                    # Read available bytes
                    if self.serial_port.in_waiting > 0:
                        buffer.extend(self.serial_port.read(self.serial_port.in_waiting))
                    
                    # Look for packet (headers 0xAA 0xFF)
                    while len(buffer) >= TELEMETRY_SIZE:
                        # Find header
                        if buffer[0] == 0xAA and buffer[1] == 0xFF:
                            # Extract packet
                            packet = buffer[:TELEMETRY_SIZE]
                            buffer = buffer[TELEMETRY_SIZE:]
                            
                            # Parse telemetry
                            self.parse_telemetry(packet)
                        else:
                            # Skip invalid byte
                            buffer.pop(0)
                
                time.sleep(0.01)  # 100 Hz check rate
                
            except Exception as e:
                self.get_logger().error(f'Telemetry read error: {e}')
                time.sleep(0.1)

    def command_timer_callback(self):
        """Send periodic commands to MCU"""
        current_time = time.time()
        
        # If no target for timeout period, send home position command
        if current_time - self.last_target_time > self.target_timeout:
            # Send home position (0, 0) = parallel to ground
            self.send_to_mcu(
                pan_delta=0.0,
                tilt_delta=0.0,
                laser_enable=True,
                laser_fire=False
            )

    def parse_telemetry(self, packet):
        """
        Parse telemetry packet from MCU
        Packet structure (32 bytes):
        - [0-1]   Headers (0xAA 0xFF)
        - [2-5]   current_world_pan (float32)
        - [6-9]   current_world_tilt (float32)
        - [10-13] encoder_pan (float32)
        - [14-17] encoder_tilt (float32)
        - [18-21] error_pan (float32)
        - [22-25] error_tilt (float32)
        - [26-27] status_flags (uint16)
        - [28-31] timestamp_ms (uint32)
        - [30-31] CRC16
        """
        try:
            # Unpack telemetry (little-endian)
            data = struct.unpack('<BBffffffHIH', packet)
            
            header1 = data[0]  # 0xAA
            header2 = data[1]  # 0xFF
            current_world_pan = data[2]
            current_world_tilt = data[3]
            encoder_pan = data[4]
            encoder_tilt = data[5]
            error_pan = data[6]
            error_tilt = data[7]
            status_flags = data[8]
            timestamp_ms = data[9]
            crc_received = data[10]
            
            # TODO: Verify CRC (optional but recommended)
            
            # Publish feedback
            fb = GimbalFeedback()
            fb.pan_deg = current_world_pan
            fb.tilt_deg = current_world_tilt
            fb.error_pan = error_pan
            fb.error_tilt = error_tilt
            fb.at_limit = bool(status_flags & 0x0003)  # Pan or tilt at limit
            fb.safe_mode = bool(status_flags & 0x0010)
            
            self.feedback_pub.publish(fb)
            
            # Log occasionally for debugging
            if timestamp_ms % 1000 < 100:  # Every ~1 second
                self.get_logger().info(
                    f'Telemetry: World[{current_world_pan:.2f}, {current_world_tilt:.2f}] '
                    f'Enc[{encoder_pan:.2f}, {encoder_tilt:.2f}] '
                    f'Err[{error_pan:.3f}, {error_tilt:.3f}]'
                )
            
        except Exception as e:
            self.get_logger().error(f'Telemetry parse error: {e}')

    """
    def send_laser_fire_command(self):
        if self.laser_fire_client.wait_for_service(timeout_sec=1.0):
            req = FireCommand.Request()
            req.duration_ms = 100  # örnek süre
            req.power_percent = 50  # örnek güç
            req.safety_token = "TOKEN"  # placeholder
            self.laser_fire_client.call_async(req)
            self.get_logger().info('Laser fire command sent')
        else:
            self.get_logger().warn('Laser fire service not available')
    """

    def calculate_crc16(self, data):
        """
        CRC-16-CCITT calculation (matches MCU implementation)
        Polynomial: 0x1021
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF  # Keep 16-bit
        return crc
    
    def parameter_callback(self, params):    
        for param in params:
            if param.name == 'centering_threshold':
                self.threshold = param.value
                self.get_logger().info(f'Parameter updated: centering_threshold = {self.threshold}')
            elif param.name == 'lock_duration':
                self.lock_duration = param.value
                self.get_logger().info(f'Parameter updated: lock_duration = {self.lock_duration}')
                
            elif param.name in ['serial_port', 'baudrate']:
                self.get_logger().info(f'Hardware setting changed: {param.name}. Resetting serial connection...')
                
                if param.name == 'serial_port':
                    self.serial_port_name = param.value
                if param.name == 'baudrate':
                    self.baud_rate_val = param.value
    
                try:
                    if self.serial_port and self.serial_port.is_open:
                        self.serial_port.close()
                    
                    self.serial_port = serial.Serial(
                        port=self.serial_port_name,
                        baudrate=self.baud_rate_val,
                        timeout=0.01
                    )
                    self.get_logger().info(f'Serial connection re-established successfully on: {self.serial_port_name}')
                except Exception as e:
                    self.get_logger().error(f'Failed to re-establish serial connection: {e}')
    
        return SetParametersResult(successful=True)
    
def main(args=None):
    rclpy.init(args=args)
    node = OperationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
