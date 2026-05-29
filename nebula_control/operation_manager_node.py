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

    # ROS-side operational modes
    MODE_SAFE   = 0
    MODE_SEARCH = 1
    MODE_LASER  = 2

    # MCU protocol mode constants (must match SerialProtocol.h)
    MCU_MODE_GROUND_LOCK = 0
    MCU_MODE_TRACKING    = 1
    MCU_MODE_JOYSTICK    = 2

    # Balloon targeting state machine
    STATE_IDLE     = 'IDLE'      # No balloons — ground lock heartbeat
    STATE_TRACKING = 'TRACKING'  # Tracking nearest balloon
    STATE_FIRING   = 'FIRING'    # Laser firing for fire_duration
    STATE_COOLDOWN = 'COOLDOWN'  # Holding position after fire

    def __init__(self):
        super().__init__('operation_manager_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('centering_threshold', 0.01)
        self.declare_parameter('fire_duration', 1.0)      # seconds laser fires
        self.declare_parameter('cooldown_duration', 1.5)  # seconds to hold after fire
        self.declare_parameter('balloon_timeout', 0.3)    # seconds without data = no balloons
        self.declare_parameter('tracking_gain', 0.3)      # 0-1: görüntü gecikmesi nedeniyle tam delta gönderilmez

        self.serial_port_name  = self.get_parameter('serial_port').value
        self.baud_rate_val     = self.get_parameter('baudrate').value
        self.threshold         = self.get_parameter('centering_threshold').value
        self.fire_duration     = self.get_parameter('fire_duration').value
        self.cooldown_duration = self.get_parameter('cooldown_duration').value
        self.balloon_timeout   = self.get_parameter('balloon_timeout').value
        self.tracking_gain     = self.get_parameter('tracking_gain').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # QoS profiles
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

        # Serial
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
        self.balloons_sub   = self.create_subscription(BalloonArray,   '/vision/balloons',    self.balloons_callback,    target_qos)
        self.rectangles_sub = self.create_subscription(RectangleArray, '/vision/rectangles',  self.rectangles_callback,  target_qos)
        self.cam_info_sub   = self.create_subscription(CameraInfo,     '/camera/camera_info', self.camera_info_callback, cam_qos)

        # Publishers
        self.gimbal_mode_pub = self.create_publisher(GimbalMode,     '/gimbal/mode',     mode_qos)
        self.feedback_pub    = self.create_publisher(GimbalFeedback, '/gimbal/feedback', feedback_qos)

        # Services
        self.set_mode_service = self.create_service(SetMode, '/gimbal/set_mode', self.set_mode_callback)

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.img_w = self.img_h = None

        # ROS-side mode
        self.current_mode = self.MODE_LASER  # TODO: change to MODE_SAFE after testing
        self.gimbal_mode_pub.publish(GimbalMode(mode=self.current_mode))
        self.get_logger().info(f'Operation Manager Node started in mode {self.current_mode}')

        # Balloon data (updated by subscriber, consumed by control_loop)
        self.last_balloons     = []
        self.last_balloon_time = 0.0

        # State machine
        self.state               = self.STATE_IDLE
        self.fire_start_time     = 0.0
        self.cooldown_start_time = 0.0

        # Hedef kilitleme — rastgele balon atlamasını önler
        self._locked_target_pos    = None   # (u_norm, v_norm) son kilitlenen balon
        self._no_balloon_ticks     = 0      # ardışık balonsuz tick sayısı
        self._no_balloon_threshold = 5      # kaç tick sonra gerçekten kayboldu sayılır (0.5s @10Hz)

        self._last_reconnect_attempt = 0.0

        # Control loop at 10 Hz — drives state machine and sends heartbeats
        self.create_timer(0.1, self.control_loop)

    # ─── Callbacks ──────────────────────────────────────────────────────────

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
        self.last_balloons     = list(msg.balloons) if msg.balloons else []
        self.last_balloon_time = time.time()

    def rectangles_callback(self, msg):
        pass  # reserved for future use

    # ─── Serial reconnect ───────────────────────────────────────────────────

    def _try_reconnect_serial(self):
        try:
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate_val,
                timeout=0.01
            )
            self.get_logger().info(f'Serial reconnected on {self.serial_port_name}')
        except Exception:
            self.serial_port = None

    # ─── State machine ──────────────────────────────────────────────────────

    def control_loop(self):
        # Auto-reconnect serial if disconnected (try every 2 seconds)
        if not self.serial_port or not self.serial_port.is_open:
            now = time.time()
            if now - self._last_reconnect_attempt > 2.0:
                self._last_reconnect_attempt = now
                self._try_reconnect_serial()
            return  # skip this tick, MCU is already in GROUND_LOCK via timeout

        if self.current_mode != self.MODE_LASER or self.fx is None:
            return

        now = time.time()
        balloon_fresh = (now - self.last_balloon_time) < self.balloon_timeout
        has_balloons  = balloon_fresh and bool(self.last_balloons)

        if self.state == self.STATE_IDLE:
            # Heartbeat: tell MCU we are alive and want ground lock
            self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_GROUND_LOCK)

            if has_balloons:
                self.state = self.STATE_TRACKING
                self.get_logger().info('IDLE -> TRACKING')

        elif self.state == self.STATE_TRACKING:
            if not has_balloons:
                self._no_balloon_ticks += 1
                if self._no_balloon_ticks >= self._no_balloon_threshold:
                    # Gerçekten kayboldu — IDLE'a dön, kilidi sıfırla
                    self.state = self.STATE_IDLE
                    self._locked_target_pos = None
                    self._no_balloon_ticks  = 0
                    self.get_logger().info('TRACKING -> IDLE: balon kayboldu')
                else:
                    # Geçici kayıp — son pozisyonda bekle (delta=0 → mevcut konumu koru)
                    self.send_to_mcu(0.0, 0.0, True, False, mode=self.MCU_MODE_TRACKING)
                return

            self._no_balloon_ticks = 0

            target = self._select_target(self.last_balloons)
            pan_delta, tilt_delta = self.norm_to_angle(target.u_norm, target.v_norm)
            is_centered = (abs(target.u_norm - 0.5) < self.threshold and
                           abs(target.v_norm - 0.5) < self.threshold)

            self.send_to_mcu(pan_delta * self.tracking_gain, tilt_delta * self.tracking_gain,
                             True, False, mode=self.MCU_MODE_TRACKING)

            if is_centered:
                self.state           = self.STATE_FIRING
                self.fire_start_time = now
                self.get_logger().info(
                    f'TRACKING -> FIRING: centered pan={pan_delta:.2f} tilt={tilt_delta:.2f}')

        elif self.state == self.STATE_FIRING:
            fire_elapsed = now - self.fire_start_time

            if fire_elapsed < self.fire_duration:
                # Keep tracking the balloon while firing; hold position if it already popped
                if has_balloons:
                    target = self._select_target(self.last_balloons)
                    pan_delta, tilt_delta = self.norm_to_angle(target.u_norm, target.v_norm)
                else:
                    pan_delta, tilt_delta = 0.0, 0.0
                self.send_to_mcu(pan_delta * self.tracking_gain, tilt_delta * self.tracking_gain,
                                 True, True, mode=self.MCU_MODE_TRACKING)
            else:
                # Fire complete — enter cooldown, laser off
                self.state               = self.STATE_COOLDOWN
                self.cooldown_start_time = now
                self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_TRACKING)
                self.get_logger().info('FIRING -> COOLDOWN')

        elif self.state == self.STATE_COOLDOWN:
            # delta=0 in TRACKING mode → MCU holds current world position
            self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_TRACKING)

            if (now - self.cooldown_start_time) >= self.cooldown_duration:
                if has_balloons:
                    # Bir sonraki balona geç — önceki kilidi sıfırla
                    self._locked_target_pos = None
                    self._no_balloon_ticks  = 0
                    self.state = self.STATE_TRACKING
                    self.get_logger().info('COOLDOWN -> TRACKING: sonraki hedefe geçiliyor')
                else:
                    self._locked_target_pos = None
                    self.state = self.STATE_IDLE
                    self.get_logger().info('COOLDOWN -> IDLE: tüm balonlar imha edildi')

    # ─── Helpers ────────────────────────────────────────────────────────────

    def _select_target(self, balloons):
        """
        Hedef balonunu seç.
        İlk kez veya kilit yoksa merkeze en yakını seç.
        Kilit varsa son bilinen pozisyona en yakını seç — tek frame kayıplarında
        farklı balona atlamayı engeller.
        """
        if not balloons:
            return None

        if self._locked_target_pos is None:
            # İlk seçim: görüntü merkezine en yakın
            target = min(balloons, key=lambda b: (b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2)
        else:
            lx, ly = self._locked_target_pos
            # Son bilinen pozisyona en yakın balonu bul
            target = min(balloons, key=lambda b: (b.u_norm - lx)**2 + (b.v_norm - ly)**2)
            dist = math.sqrt((target.u_norm - lx)**2 + (target.v_norm - ly)**2)
            # Çok uzaksa (büyük sıçrama) merkeze en yakına geri dön
            if dist > 0.25:
                target = min(balloons, key=lambda b: (b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2)

        self._locked_target_pos = (target.u_norm, target.v_norm)
        return target

    def norm_to_angle(self, u_norm, v_norm):
        # Gerçek kalibrasyon değerleri gelince camera_info_callback'ten otomatik dolar:
        #   fx = K[0], fy = K[4], cx = K[2], cy = K[5]
        # Kalibrasyon yapılana kadar fallback:
        #   cx = img_w / 2  (görüntü yatay merkezi)
        #   cy = img_h / 2  (görüntü dikey merkezi)
        #   fx = img_w      (~53° HFoV varsayımı: focal_length ≈ width / (2*tan(HFoV/2)))
        #   fy = img_w      (kare piksel varsayımı, fy = fx)
        cx = self.cx if self.cx is not None else self.img_w * 0.5
        cy = self.cy if self.cy is not None else self.img_h * 0.5
        fx = self.fx if self.fx is not None else self.img_w
        fy = self.fy if self.fy is not None else self.img_w

        # Piksel → normalize kamera koordinatı → açı (derece)
        #   x = (u_px - cx) / fx
        #   y = (v_px - cy) / fy
        #   pan  =  atan(x)  (sağ pozitif)
        #   tilt = -atan(y)  (yukarı pozitif, görüntü y ekseni aşağı yönlü olduğu için eksi)
        u = u_norm * self.img_w
        v = v_norm * self.img_h
        x = (u - cx) / fx
        y = (v - cy) / fy
        pan  =  math.degrees(math.atan(x))
        tilt = -math.degrees(math.atan(y))
        return pan, tilt

    def send_to_mcu(self, pan_delta, tilt_delta, laser_enable, laser_fire,
                    mode=None, ff_pan=0.0, ff_tilt=0.0):
        """
        Binary packet structure (24 bytes total):
        - [0-1]   Header: 0xAA 0xFF
        - [2-5]   pan_delta (float32)   — TRACKING: degrees off-center
        - [6-9]   tilt_delta (float32)  — TRACKING: degrees off-center
        - [10-13] feedforward_vel_pan (float32)  — JOYSTICK: deg/s
        - [14-17] feedforward_vel_tilt (float32) — JOYSTICK: deg/s
        - [18]    laser_enable (uint8)
        - [19]    laser_fire (uint8)
        - [20]    mode (uint8): 0=GROUND_LOCK, 1=TRACKING, 2=JOYSTICK
        - [21]    reserved (uint8, 0x00)
        - [22-23] CRC16 (covers bytes 0-21)
        """
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return

        if mode is None:
            mode = self.MCU_MODE_TRACKING

        try:
            packet_data = struct.pack(
                '<BBffffBBBB',
                0xAA, 0xFF,
                float(pan_delta), float(tilt_delta),
                float(ff_pan), float(ff_tilt),
                int(laser_enable), int(laser_fire),
                int(mode), 0x00
            )
            crc    = self.calculate_crc16(packet_data)
            packet = packet_data + struct.pack('<H', crc)
            self.serial_port.write(packet)
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.serial_port = None  # trigger reconnect in control_loop

    def telemetry_reader_loop(self):
        buffer = bytearray()
        TELEMETRY_SIZE = 32

        while rclpy.ok():
            try:
                if self.serial_port and self.serial_port.is_open:
                    if self.serial_port.in_waiting > 0:
                        buffer.extend(self.serial_port.read(self.serial_port.in_waiting))

                    while len(buffer) >= TELEMETRY_SIZE:
                        if buffer[0] == 0xAA and buffer[1] == 0xFF:
                            packet = bytes(buffer[:TELEMETRY_SIZE])
                            buffer = buffer[TELEMETRY_SIZE:]
                            self.parse_telemetry(packet)
                        else:
                            buffer.pop(0)

                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Telemetry read error: {e}')
                time.sleep(0.1)

    def parse_telemetry(self, packet):
        """
        Parse telemetry packet from MCU (32 bytes).
        Layout: BB + 6×float + uint16 + uint32(CRC overwrites last 2 bytes — known issue)
        Safe to unpack first 28 bytes: '<BBffffffH'
        """
        try:
            data = struct.unpack('<BBffffffH', packet[:28])
            current_world_pan  = data[2]
            current_world_tilt = data[3]
            error_pan          = data[6]
            error_tilt         = data[7]
            status_flags       = data[8]

            fb = GimbalFeedback()
            fb.pan_deg   = current_world_pan
            fb.tilt_deg  = current_world_tilt
            fb.error_pan = error_pan
            fb.error_tilt = error_tilt
            fb.at_limit  = bool(status_flags & 0x0003)
            fb.safe_mode = bool(status_flags & 0x0010)
            self.feedback_pub.publish(fb)
        except Exception as e:
            self.get_logger().error(f'Telemetry parse error: {e}')

    def calculate_crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'centering_threshold':
                self.threshold = param.value
            elif param.name == 'fire_duration':
                self.fire_duration = param.value
            elif param.name == 'cooldown_duration':
                self.cooldown_duration = param.value
            elif param.name == 'balloon_timeout':
                self.balloon_timeout = param.value
            elif param.name == 'tracking_gain':
                self.tracking_gain = param.value
            elif param.name in ['serial_port', 'baudrate']:
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
                    self.get_logger().info(f'Serial reconnected on {self.serial_port_name}')
                except Exception as e:
                    self.get_logger().error(f'Serial reconnect failed: {e}')
            self.get_logger().info(f'Parameter updated: {param.name} = {param.value}')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = OperationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
