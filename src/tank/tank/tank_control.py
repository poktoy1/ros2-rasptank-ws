
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tank.action_turn_client import ActionTurnClient
from tank_interfaces.action import Turn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class TankControl(Node):

    def __init__(self, action_client: ActionTurnClient):
        super().__init__('tank_control')
        self._sonar_lock = threading.Lock()
        self._sonar_msg: Range
        self._action_client: ActionTurnClient = action_client
        self._sonar_subscriber = self.create_subscription(
            msg_type=Range,
            topic='robot/sonar/collision',
            callback=self.listener_callback,
            qos_profile=10
        )
        self._sonar_subscriber

        self._twist_publisher = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10
        )

    def listener_callback(self, msg: Range):

        self._sonar_msg = msg
        with self._sonar_lock:
            twist = Twist()
            if msg.range < msg.min_range:

                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._twist_publisher.publish(twist)
                goal = Turn.Goal()
                goal.angular_velocity = -0.8
                self._action_client.send_goal(goal=goal)
            else:
                twist.linear.x = 1.0
                twist.angular.z = 0.0
                self._twist_publisher.publish(twist)

    def clean_up(self):
        self._sonar_subscriber.destroy()
        self._twist_publisher.destroy()
        super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = ActionTurnClient()
        tank_control = TankControl(action_client=action_client)
        executors = MultiThreadedExecutor()
        executors.add_node(action_client)
        executors.add_node(tank_control)
        executors.spin()

    finally:
        action_client.clean_up()
        tank_control.clean_up()
        executors.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
