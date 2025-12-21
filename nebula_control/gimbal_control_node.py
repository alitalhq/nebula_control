import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo
from nebula_interfaces.msg import GimbalCommand, GimbalFeedback

import serial
import struct
import math


class GimbalControllerNode(Node):

    def __init__(self):
        super().__init__('gimbal_controller_node')

        # ---------------- QoS ----------------
        cmd_qos = QoSProfile(depth=10)
        cmd_qos.reliability = ReliabilityPolicy.RELIABLE
        cmd_qos.history = HistoryPolicy.KEEP_LAST

        feedback_qos = QoSProfile(depth=1)
        feedback_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        feedback_qos.history = HistoryPolicy.KEEP_LAST

        cam_qos = QoSProfile(depth=1)
        cam_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        cam_qos.history = HistoryPolicy.KEEP_LAST

        # -------------- Subscribers ----------
        self.cmd_sub = self.create_subscription(
            GimbalCommand,
            '/gimbal/command',
            self.command_callback,
            cmd_qos
        )

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/info',
            self.camera_info_callback,
            cam_qos
        )

        # -------------- Publisher ------------
        self.feedback_pub = self.create_publisher(
            GimbalFeedback,
            '/gimbal/feedback',
            feedback_qos
        )

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_w = None
        self.img_h = None

        # -------------- Serial ---------------
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.01
            )
            self.get_logger().info('Serial connected')
        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f'Serial error: {e}')

    # =========================================================

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.img_w = msg.width
        self.img_h = msg.height

    # =========================================================

    def command_callback(self, msg: GimbalCommand):

        if self.fx is None:
            return  # kamera bilgisi gelmeden işlem yok

        # ---------- MODE_POS ----------
        if msg.mode == GimbalCommand.MODE_POS:
            pan_deg, tilt_deg = self.norm_to_angle(
                msg.target_u_norm,
                msg.target_v_norm
            )

        else:
            return  # MODE_VEL şimdilik yok

        # ---------- SERIAL PACKET ----------
        self.send_to_mcu(
            pan_deg=pan_deg,
            tilt_deg=tilt_deg,
            laser_enable=msg.laser_enable,
            laser_fire=msg.laser_fire_request
        )

        # ---------- Feedback (placeholder) ----------
        fb = GimbalFeedback()
        fb.current_pan_angle = pan_deg
        fb.current_tilt_angle = tilt_deg
        self.feedback_pub.publish(fb)

    # =========================================================

    def norm_to_angle(self, u_norm, v_norm):
        """
        Kamera merkezine göre açı hesabı
        """
        u = u_norm * self.img_w
        v = v_norm * self.img_h

        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        pan = math.degrees(math.atan(x))
        tilt = -math.degrees(math.atan(y))

        return pan, tilt

    # =========================================================

    def send_to_mcu(self, pan_deg, tilt_deg, laser_enable, laser_fire):
        """
        Binary packet:
        float pan
        float tilt
        uint8 laser_enable
        uint8 laser_fire
        """
        if not self.serial_port:
            return

        try:
            packet = struct.pack(
                '<ffBB',
                float(pan_deg),
                float(tilt_deg),
                int(laser_enable),
                int(laser_fire)
            )
            self.serial_port.write(packet)
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
