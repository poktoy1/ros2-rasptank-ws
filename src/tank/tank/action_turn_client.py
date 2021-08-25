
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from tank_interfaces.action import Turn
from action_msgs.msg import GoalStatus


class ActionTurnClient(Node):

    def __init__(self):
        super().__init__('action_turn_client')
        self._action_client = ActionClient(
            node=self,
            action_type=Turn,
            action_name='Turn'
        )

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(
            feedback))

    def get_result_callback(self, future: Future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        # print('shutting down')
        # rclpy.try_shutdown()

    def goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().log('Goal rejected')
            return
        self.get_logger().info('Goal Accepted')
        self._get_result_future: Future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(callback=self.get_result_callback)

    def send_goal(self, goal: Turn.Goal):
        self.get_logger().info('Waiting for Action Server')
        result = self._action_client.wait_for_server(timeout_sec=5)
        if(result):
            self._send_goal_future = self._action_client.send_goal_async(
                goal=goal,
                feedback_callback=self.feedback_callback
            )
            self._send_goal_future.add_done_callback(
                callback=self.goal_response_callback
            )

    def clean_up(self):
        self._action_client.destroy()
        super().destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)
        action_client = ActionTurnClient()
        goal = Turn.Goal()
        goal.angular_velocity = 1.0
        action_client.send_goal(goal=goal)
        rclpy.spin_once(action_client)

    finally:
        rclpy.try_shutdown()
        print('shutdown')


if __name__ == '__main__':
    main()
