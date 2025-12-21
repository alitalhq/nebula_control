import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy

from nebula_interfaces.msg import BalloonArray, RectangleArray, GimbalFeedback, GimbalCommand, GimbalMode
from nebula_interfaces.srv import SetMode

class OperationManagerNode(Node):

    # Define operational modes
    MODE_SAFE = 0
    MODE_SEARCH = 1
    MODE_LASER = 2

    def __init__(self):
        super().__init__('operation_manager_node')

        cmd_qos = QoSProfile(depth=10)
        cmd_qos.reliability=ReliabilityPolicy.RELIABLE
        cmd_qos.history=HistoryPolicy.KEEP_LAST
    
        feedback_qos = QoSProfile(depth=1)
        feedback_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        target_qos = QoSProfile(depth=1)
        target_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = QoSReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscriptions
        self.balloons_sub = self.create_subscription(BalloonArray, '/vision/balloons', self.balloons_callback, target_qos)
        self.balloons_sub = self.create_subscription(RectangleArray, '/vision/rectangles', self.rectangles_callback, target_qos)
        self.gimbal_feedback_sub = self.create_subscription(GimbalFeedback, '/gimbal/feedback', self.gimbal_feedback_callback, feedback_qos)
        
        # Publishers
        self.gimbal_command_pub = self.create_publisher(GimbalCommand, '/gimbal/command', cmd_qos)
        self.gimbal_mode_pub = self.create_publisher(GimbalMode, '/gimbal/mode', mode_qos)

        """
        # Clients
        self.laser_fire_client = self.create_client(FireCommand, '/laser/fire')
        while not self.laser_fire_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('laser_fire service not available, waiting again...')
        """

        # Services
        self.set_mode_service = self.create_service(SetMode, '/gimbal/set_mode', self.set_mode_callback)

        self.current_mode = self.MODE_LASER #TESTLERDEN SONRA MODE_SAFE YAPMAYI UNUTMA
        self.gimbal_mode_pub.publish(GimbalMode(mode=self.current_mode))
        self.get_logger().info(f'Operation Manager Node has been started in {self.current_mode} mode.')

        # Internal state variables
        self.last_gimbal_feedback = None
        self.last_balloons = None
        self.last_rectangles = None

        self.timer = self.create_timer(0.1, self.periodic_task)

    def balloons_callback(self, msg):
        self.last_balloons = msg

    def rectangles_callback(self, msg):
        self.last_rectangles = msg

    def gimbal_feedback_callback(self, msg):
        self.last_gimbal_feedback = msg
        #self.get_logger().info(f'Received gimbal feedback: Pan={msg.current_pan_angle:.2f}, Tilt={msg.current_tilt_angle:.2f}')

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
    
    def send_gimbal_command(self, u_norm, v_norm):
        cmd = GimbalCommand()

        cmd.mode = GimbalCommand.MODE_POS
        cmd.ref_frame = GimbalCommand.REF_BODY

        cmd.target_u_norm = float(u_norm)
        cmd.target_v_norm = float(v_norm)

        cmd.pan_deg = 0.0   # gimbal node will convert u_norm -> pan
        cmd.tilt_deg = 0.0  # gimbal node will convert v_norm -> tilt

        cmd.laser_enable = True
        cmd.laser_fire_request = True

        cmd.priority = 1
        cmd.requester = "operation_manager"
        
        self.gimbal_command_pub.publish(cmd)

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


    # ------------------ Main periodic task ------------------
    def periodic_task(self):
        if self.current_mode == self.MODE_SAFE:
            # TODO: park gimbal, laser off
            pass

        elif self.current_mode == self.MODE_SEARCH:
            # TODO: search mode logic
            pass

        elif self.current_mode == self.MODE_LASER:
            # Select closest balloon
            target = None
            if self.last_balloons and len(self.last_balloons.balloons) > 0:
                # choose balloon with max confidence or closest to center
                target = min(
                    self.last_balloons.balloons,
                    key=lambda b: ((b.u_norm - 0.5)**2 + (b.v_norm - 0.5)**2)
                )

            if target:
                # Send gimbal command
                self.send_gimbal_command(target.u_norm, target.v_norm)
                # Fire laser if roughly centered
                if abs(target.u_norm - 0.5) < 0.005 and abs(target.v_norm - 0.5) < 0.005:
                    #self.send_laser_fire_command()
                    self.get_logger().info("ATEŞ!")

def main(args=None):
    rclpy.init(args=args)
    node = OperationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()