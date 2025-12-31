import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import CameraInfo
from nebula_interfaces.msg import BalloonArray, RectangleArray, GimbalFeedback, GimbalMode
from nebula_interfaces.srv import SetMode

import serial
import struct
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
            #return   #testlerden sonra return aktif edilmeli
        self.is_waiting = False

        if self.current_mode != self.MODE_LASER or not msg.balloons or self.fx is None:
            return
        
        target = min(msg.balloons, key=lambda b: ((b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2))

        pan_delta, tilt_delta = self.norm_to_angle(target.u_norm, target.v_norm)
        
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
        Binary packet structure (Total 12 bytes):
        - Header 1: 0xAA (uint8)
        - Header 2: 0xFF (uint8)
        - Pan Delta: float32 (4 bytes)
        - Tilt Delta: float32 (4 bytes)
        - Laser Enable: uint8 (1 byte)
        - Laser Fire: uint8 (1 byte)
        Format string: '<BBffBB'
        """
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            packet = struct.pack(
                '<BBffBB',
                0xAA,               # Header 1
                0xFF,               # Header 2
                float(pan_delta),    # 4 byte float
                float(tilt_delta),   # 4 byte float
                int(laser_enable),   # 1 byte
                int(laser_fire)      # 1 byte
            )
            self.serial_port.write(packet)
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


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
