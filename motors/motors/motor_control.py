import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TwistToMotors(Node):

    def __init__(self):
        super().__init__('twist_to_motors')

        self.declare_parameter('base_width', 0.31)
        self.declare_parameter('rate', 5)
        self.declare_parameter('timeout_ticks', 2)

        self.w = self.get_parameter('base_width').value
        self.rate_value = self.get_parameter('rate').value
        self.timeout_ticks = self.get_parameter('timeout_ticks').value

        self.pub_lmotor = self.create_publisher(Float32, 'lwheel_vtarget', 10)
        self.pub_rmotor = self.create_publisher(Float32, 'rwheel_vtarget', 10)

        self.subscription = self.create_subscription(
            Twist,
            '/diffbot_controller/cmd_vel',
            self.twistCallback,
            10
        )

        self.dx = 0.0
        self.dr = 0.0
        self.ticks_since_target = self.timeout_ticks

        self.timer = self.create_timer(1.0 / self.rate_value, self.spinOnce)

        self.get_logger().info('TwistToMotors node started')

    def spinOnce(self):
        # Default to zero velocity
        left = 0.0
        right = 0.0

        # Check if we are actively receiving commands OR if the last command was non-zero
        if self.ticks_since_target < self.timeout_ticks:
            # Check if the command itself is non-zero
            if abs(self.dx) > 0.001 or abs(self.dr) > 0.001: # Use a small tolerance for floating point
                # Calculate target velocities ONLY if command is non-zero
                right = self.dx + self.dr * self.w / 2.0 # Corrected kinematics assumed
                left  = self.dx - self.dr * self.w / 2.0 # Corrected kinematics assumed
            # else: dx and dr are zero, left/right remain 0.0

            self.ticks_since_target += 1
        # else: Timeout occurred, left/right remain 0.0

        # Always publish the current target (calculated, zero from zero input, or zero from timeout)
        self.pub_lmotor.publish(Float32(data=left))
        self.pub_rmotor.publish(Float32(data=right))

        # Add logging to see exactly what's being published
        self.get_logger().debug(f'Publishing Target Vel - Left: {left:.2f}, Right: {right:.2f}, dx: {self.dx:.2f}, dr: {self.dr:.2f}, Ticks: {self.ticks_since_target}')

    def twistCallback(self, msg):
        # Reset timeout counter
        self.ticks_since_target = 0
        # Store received velocities
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        # Log received command
        self.get_logger().debug(f'Received Twist: dx: {self.dx:.2f}, dr: {self.dr:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToMotors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
