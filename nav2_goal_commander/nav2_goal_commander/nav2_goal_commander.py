import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class Nav2GoalCommander(Node):

    def __init__(self):
        super().__init__('nav2_goal_commander')

        self.current_waypoint = 0

        ##this values were chosen using the command:
        ##ros2 topic echo /clicked_point --once
        ##and using the Publish Point Feature at rviz2
        self.coordinates = [
            ##(0.019213, 2.008589)
            (-1.795918, -0.750146),
            (0.736136, 2.239788),
            (0.226881, -1.910624)
        ]

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info(f'Mission initialized with {len(self.coordinates)} coordinates.')
        
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):

        if not self.action_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 action server...')
            return

        self.timer.cancel()

        if self.current_waypoint >= len(self.coordinates):
            self.get_logger().info('Mission completed!')
            return

        x, y = self.coordinates[self.current_waypoint]

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending waypoint {self.current_waypoint + 1}: '
            f'x={x}, y={y}'
        )

        future = self.action_client.send_goal_async(goal,feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Coordinate {self.current_waypoint + 1} rejected.')
            return

        self.get_logger().info(f'Coordinate {self.current_waypoint + 1} accepted.')

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Coordinate {self.current_waypoint + 1} - '
            f'Distance remaining: '
            f'{feedback.distance_remaining:.2f} m'
        )

    def result_callback(self, future):
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Coordinate {self.current_waypoint + 1} reached!')
            self.current_waypoint += 1

            if self.current_waypoint < len(self.coordinates):
                self.get_logger().info('Moving to the next waypoint...')
                self.send_goal()

            else:
                self.get_logger().info('All coordinates reached. Mission completed!')

        else:
            self.get_logger().error(
                f'Navigation failed at waypoint '
                f'{self.current_waypoint + 1}. '
                f'Status: {result.status}'
            )

            self.get_logger().error('Mission aborted.')

def main(args=None):
    rclpy.init(args=args)

    node = Nav2GoalCommander()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()