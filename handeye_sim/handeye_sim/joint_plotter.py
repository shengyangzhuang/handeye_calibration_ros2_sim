import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

# Use Agg backend for non-interactive plotting
plt.switch_backend('Agg')

class JointPlotter(Node):
    def __init__(self):
        super().__init__('joint_plotter')
        self.subscription = self.create_subscription(
            JointState,
            '/lbr/joint_states',
            self.joint_state_callback,
            10)
        self.joint_positions = []
        self.joint_names = None

    def joint_state_callback(self, msg):
        if self.joint_names is None:
            self.joint_names = msg.name  # Store joint names once
        self.joint_positions.append(msg.position)
        if len(self.joint_positions) > 100:
            self.joint_positions.pop(0)
        self.plot_joint_positions()

    def plot_joint_positions(self):
        plt.clf()
        for idx, joint_pos in enumerate(zip(*self.joint_positions)):
            plt.plot(joint_pos, label=self.joint_names[idx])
        plt.legend()

    def save_plot(self):
        self.plot_joint_positions()  # Ensure the latest data is plotted
        plt.savefig("src/handeye_sim/resource/joint_positions_plot.png")
        self.get_logger().info("Plot saved as 'joint_positions_plot.png'")

def main(args=None):
    rclpy.init(args=args)
    node = JointPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down and saving plot...")
        node.save_plot()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
