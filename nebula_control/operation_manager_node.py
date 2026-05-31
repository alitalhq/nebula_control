import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
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

    MODE_SAFE   = 0
    MODE_SEARCH = 1
    MODE_LASER  = 2

    MCU_MODE_GROUND_LOCK = 0
    MCU_MODE_TRACKING    = 1
    MCU_MODE_JOYSTICK    = 2

    STATE_IDLE      = 'IDLE'       # bekleme, balon yok
    STATE_CENTERING = 'CENTERING'  # PX4 mavi dikdörtgeni frame merkezine getiriyor
    STATE_ENGAGING  = 'ENGAGING'   # gimbal balona kilitlenmeye çalışıyor
    STATE_LOCKED    = 'LOCKED'     # kilit sağlandı, ateşe hazır
    STATE_FIRING    = 'FIRING'     # lazer açık
    STATE_VERIFYING = 'VERIFYING'  # ateş sonrası balon kaldı mı kontrol
    STATE_DONE      = 'DONE'       # görev tamamlandı

    def __init__(self):
        super().__init__('operation_manager_node')

        self.declare_parameter('serial_port',          '/dev/ttyACM0')
        self.declare_parameter('baudrate',             921600)
        self.declare_parameter('centering_threshold',  0.05)   # frame fraksiyonu, dikdörtgen merkez toleransı
        self.declare_parameter('lock_angle_threshold', 1.0)    # derece, kilit için max hata
        self.declare_parameter('lock_duration',        0.3)    # saniye, kilidi onaylamak için bekleme süresi
        self.declare_parameter('fire_duration',        1.5)    # saniye, lazer açık kalma süresi
        self.declare_parameter('verify_timeout',       3.0)    # saniye, VERIFYING'de balon bekleme süresi
        self.declare_parameter('balloon_timeout',      0.3)    # saniye, veri kesilince balon yok sayılır
        self.declare_parameter('tracking_gain',        0.3)

        self.serial_port_name     = self.get_parameter('serial_port').value
        self.baud_rate_val        = self.get_parameter('baudrate').value
        self.threshold            = self.get_parameter('centering_threshold').value
        self.lock_angle_threshold = self.get_parameter('lock_angle_threshold').value
        self.lock_duration        = self.get_parameter('lock_duration').value
        self.fire_duration        = self.get_parameter('fire_duration').value
        self.verify_timeout       = self.get_parameter('verify_timeout').value
        self.balloon_timeout      = self.get_parameter('balloon_timeout').value
        self.tracking_gain        = self.get_parameter('tracking_gain').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        best_effort_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        reliable_latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate_val,
                timeout=0.01)
            self.get_logger().info('Serial connected')
        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f'Serial error: {e}')

        self.balloons_sub   = self.create_subscription(BalloonArray,   '/vision/balloons',    self.balloons_callback,    best_effort_qos)
        self.rectangles_sub = self.create_subscription(RectangleArray, '/vision/rectangles',  self.rectangles_callback,  best_effort_qos)
        self.cam_info_sub   = self.create_subscription(CameraInfo,     '/camera/camera_info', self.camera_info_callback, best_effort_qos)

        self.gimbal_mode_pub = self.create_publisher(GimbalMode,     '/gimbal/mode',     reliable_latch_qos)
        self.feedback_pub    = self.create_publisher(GimbalFeedback, '/gimbal/feedback', best_effort_qos)

        self.set_mode_service = self.create_service(SetMode, '/gimbal/set_mode', self.set_mode_callback)

        self.fx = self.fy = self.cx = self.cy = None
        self.img_w = self.img_h = None

        self.current_mode = self.MODE_LASER
        self.gimbal_mode_pub.publish(GimbalMode(mode=self.current_mode))

        self.last_balloons       = []
        self.last_balloon_time   = 0.0
        self.last_rectangles     = []
        self.last_rectangle_time = 0.0

        self.state             = self.STATE_IDLE
        self.lock_start_time   = 0.0
        self.fire_start_time   = 0.0
        self.verify_start_time = 0.0

        self._locked_target_pos    = None
        self._no_balloon_ticks     = 0
        self._no_balloon_threshold = 5

        self._last_reconnect_attempt = 0.0

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('OperationManagerNode started')

    # ─── Callbacks ──────────────────────────────────────────────────────────

    def camera_info_callback(self, msg):
        self.fx    = msg.k[0]
        self.fy    = msg.k[4]
        self.cx    = msg.k[2]
        self.cy    = msg.k[5]
        self.img_w = msg.width
        self.img_h = msg.height

    def balloons_callback(self, msg):
        self.last_balloons     = list(msg.balloons) if msg.balloons else []
        self.last_balloon_time = time.time()

    def rectangles_callback(self, msg):
        self.last_rectangles     = list(msg.rectangles) if msg.rectangles else []
        self.last_rectangle_time = time.time()

    def set_mode_callback(self, request, response):
        if request.mode in [self.MODE_SAFE, self.MODE_SEARCH, self.MODE_LASER]:
            self.current_mode = request.mode
            self.gimbal_mode_pub.publish(GimbalMode(mode=self.current_mode))
            response.success = True
            response.message = f'Mode set to {self.current_mode}'
            self.get_logger().info(f'Mode changed to: {self.current_mode}')
        else:
            response.success = False
            response.message = f'Invalid mode: {request.mode}'
        return response

    # ─── Serial reconnect ───────────────────────────────────────────────────

    def _try_reconnect_serial(self):
        try:
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate_val,
                timeout=0.01)
            self.get_logger().info(f'Serial reconnected on {self.serial_port_name}')
        except Exception:
            self.serial_port = None

    # ─── State machine ──────────────────────────────────────────────────────

    def control_loop(self):
        if not self.serial_port or not self.serial_port.is_open:
            now = time.time()
            if now - self._last_reconnect_attempt > 2.0:
                self._last_reconnect_attempt = now
                self._try_reconnect_serial()
            return

        if self.current_mode != self.MODE_LASER or self.img_w is None:
            self.get_logger().warn(f'control_loop early return: mode={self.current_mode} img_w={self.img_w}', throttle_duration_sec=5.0)
            return

        now = time.time()
        balloon_fresh   = (now - self.last_balloon_time)   < self.balloon_timeout
        rectangle_fresh = (now - self.last_rectangle_time) < self.balloon_timeout
        has_balloons    = balloon_fresh and bool(self.last_balloons)
        blue_rect       = self._find_blue_rectangle() if rectangle_fresh else None

        self.get_logger().info(
            f'[DBG] state={self.state} has_balloons={has_balloons} '
            f'n_balloons={len(self.last_balloons)} balloon_age={now - self.last_balloon_time:.2f}s',
            throttle_duration_sec=2.0)

        if self.state == self.STATE_IDLE:
            self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_GROUND_LOCK)
            if has_balloons:
                self._locked_target_pos = None
                self._no_balloon_ticks  = 0
                self._transition(self.STATE_ENGAGING)  # CENTERING PX4 hazır olunca araya girer

        elif self.state == self.STATE_ENGAGING:
            if not has_balloons:
                self._no_balloon_ticks += 1
                if self._no_balloon_ticks >= self._no_balloon_threshold:
                    self._transition(self.STATE_VERIFYING)
                else:
                    self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_TRACKING)
                return

            self._no_balloon_ticks = 0
            target = self._select_target(self.last_balloons)
            pan_deg, tilt_deg = self.norm_to_angle(target.u_norm, target.v_norm)
            self.send_to_mcu(pan_deg * self.tracking_gain, tilt_deg * self.tracking_gain,
                             True, False, mode=self.MCU_MODE_TRACKING)

            locked = abs(pan_deg) < self.lock_angle_threshold and abs(tilt_deg) < self.lock_angle_threshold
            if locked:
                if self.lock_start_time == 0.0:
                    self.lock_start_time = now
                elif now - self.lock_start_time >= self.lock_duration:
                    self._transition(self.STATE_LOCKED)
            else:
                self.lock_start_time = 0.0

        elif self.state == self.STATE_LOCKED:
            self._transition(self.STATE_FIRING)

        elif self.state == self.STATE_FIRING:
            if has_balloons:
                target = self._select_target(self.last_balloons)
                pan_deg, tilt_deg = self.norm_to_angle(target.u_norm, target.v_norm)
            else:
                pan_deg, tilt_deg = 0.0, 0.0
            self.send_to_mcu(pan_deg * self.tracking_gain, tilt_deg * self.tracking_gain,
                             True, True, mode=self.MCU_MODE_TRACKING)
            if now - self.fire_start_time >= self.fire_duration:
                self._transition(self.STATE_VERIFYING)

        elif self.state == self.STATE_VERIFYING:
            self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_GROUND_LOCK)
            if has_balloons:
                self._locked_target_pos = None
                self._no_balloon_ticks  = 0
                self._transition(self.STATE_ENGAGING)
            elif now - self.verify_start_time >= self.verify_timeout:
                self._transition(self.STATE_DONE)

        elif self.state == self.STATE_DONE:
            self.send_to_mcu(0.0, 0.0, False, False, mode=self.MCU_MODE_GROUND_LOCK)
            self.get_logger().info('Mission complete — all balloons neutralized')
            self._send_px4_mission_complete()
            self._transition(self.STATE_IDLE)

    def _transition(self, new_state):
        self.get_logger().info(f'[SM] {self.state} -> {new_state}')
        now = time.time()
        if new_state == self.STATE_FIRING:
            self.fire_start_time = now
        elif new_state == self.STATE_VERIFYING:
            self.verify_start_time = now
        elif new_state == self.STATE_ENGAGING:
            self.lock_start_time = 0.0
        self.state = new_state

    # ─── Helpers ────────────────────────────────────────────────────────────

    def _find_blue_rectangle(self):
        blue = [r for r in self.last_rectangles if r.color_label == 'blue']
        if not blue:
            return None
        return max(blue, key=lambda r: r.area_norm)

    def _send_px4_centering(self, err_x, err_y):
        # TODO: PX4 velocity command — mavi dikdörtgeni frame merkezine getir
        pass

    def _send_px4_mission_complete(self):
        # TODO: PX4'e görev tamamlandı bildir, otonom seyre devam et
        pass

    def _select_target(self, balloons):
        if not balloons:
            return None
        if self._locked_target_pos is None:
            target = min(balloons, key=lambda b: (b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2)
        else:
            lx, ly = self._locked_target_pos
            target = min(balloons, key=lambda b: (b.u_norm - lx)**2 + (b.v_norm - ly)**2)
            if math.sqrt((target.u_norm - lx)**2 + (target.v_norm - ly)**2) > 0.25:
                target = min(balloons, key=lambda b: (b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2)
        self._locked_target_pos = (target.u_norm, target.v_norm)
        return target

    def norm_to_angle(self, u_norm, v_norm):
        cx = self.cx if self.cx is not None else self.img_w * 0.5
        cy = self.cy if self.cy is not None else self.img_h * 0.5
        fx = self.fx if self.fx is not None else self.img_w
        fy = self.fy if self.fy is not None else self.img_w
        u = u_norm * self.img_w
        v = v_norm * self.img_h
        pan  =  math.degrees(math.atan((u - cx) / fx))
        tilt = -math.degrees(math.atan((v - cy) / fy))
        return pan, tilt

    def send_to_mcu(self, pan_delta, tilt_delta, laser_enable, laser_fire,
                    mode=None, ff_pan=0.0, ff_tilt=0.0):
        if not self.serial_port or not self.serial_port.is_open:
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
                int(mode), 0x00)
            crc    = self.calculate_crc16(packet_data)
            packet = packet_data + struct.pack('<H', crc)
            self.serial_port.write(packet)
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self.serial_port = None

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
        try:
            data = struct.unpack('<BBffffffH', packet[:28])
            fb           = GimbalFeedback()
            fb.pan_deg   = data[2]
            fb.tilt_deg  = data[3]
            fb.error_pan  = data[6]
            fb.error_tilt = data[7]
            status_flags  = data[8]
            fb.at_limit   = bool(status_flags & 0x0003)
            fb.safe_mode  = bool(status_flags & 0x0010)
            self.feedback_pub.publish(fb)
        except Exception as e:
            self.get_logger().error(f'Telemetry parse error: {e}')

    def calculate_crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
        return crc

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'centering_threshold':
                self.threshold = param.value
            elif param.name == 'lock_angle_threshold':
                self.lock_angle_threshold = param.value
            elif param.name == 'lock_duration':
                self.lock_duration = param.value
            elif param.name == 'fire_duration':
                self.fire_duration = param.value
            elif param.name == 'verify_timeout':
                self.verify_timeout = param.value
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
                        timeout=0.01)
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
