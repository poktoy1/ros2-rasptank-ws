
import threading
import rclpy
from rclpy.node import Node
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
        # goal = Turn.Goal()
        # goal.angular_velocity = -1.0
        # self._action_client.send_goal(goal=goal)
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
        # self.get_logger().info('sonar range:{0}'.format(msg.range))
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
        

def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = ActionTurnClient()
        tank_control = TankControl(action_client=action_client)
       
        rclpy.spin(tank_control)

    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
