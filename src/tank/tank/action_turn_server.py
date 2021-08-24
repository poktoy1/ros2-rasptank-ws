
import time
import threading
import rclpy
from rclpy.action.server import ServerGoalHandle

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tank_interfaces.action import Turn
from sensor_msgs.msg import Range


class ActionTurnServer(Node):

    def __init__(self):
        super().__init__('action_turn_server')
        self._goal_handle: ServerGoalHandle = None
        self._goal_lock = threading.Lock()
        self._sonar_msg: Range
        self._action_server = ActionServer(
            node=self,
            action_type=Turn,
            action_name='Turn',
            callback_group=MutuallyExclusiveCallbackGroup(),
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback

        )

        self._sonar_subscriber = self.create_subscription(
            msg_type=Range,
            topic='robot/sonar/collision',
            callback=self.listener_callback,
            qos_profile=10
        )
        self._sonar_subscriber

    def listener_callback(self, msg: Range):
        # self.get_logger().info('sonar range:{0}'.format(msg.range))
        self._sonar_msg = msg

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Turn.Feedback()

        for i in range(1, 10):
            if (not goal_handle.is_active):
                self.get_logger().info('Goal Aborted')
                return Turn.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Turn.Result()
            if(self._sonar_msg.range < self._sonar_msg.min_range):
                feedback_msg.partial_angular_velocity.append(-1.0)
                goal_handle.publish_feedback(feedback=feedback_msg)
            else:
                break
            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg))
            time.sleep(1)

        goal_handle.succeed()
        result = Turn.Result()
        result.angular_velocity_result = feedback_msg.partial_angular_velocity
        self.get_logger().info('Returning result: {0}'.format(result))
        return result

    def goal_callback(self, goal_request):
        self.get_logger().info(f'received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self._goal_lock:
            if (self._goal_handle is not None and self._goal_handle.is_active):
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        self._goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


def main(args=None):
    try:
        rclpy.init(args=args)
        action_server = ActionTurnServer()
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        try:
            print('starting executor spin')
            executor.spin()
        finally:
            action_server.destroy()
            print('destroy action server')
    finally:
        rclpy.try_shutdown()
        print('rclpy shutdown')


if __name__ == '__main__':
    main()
