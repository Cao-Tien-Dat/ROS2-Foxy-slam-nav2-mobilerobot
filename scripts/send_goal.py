import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
import random
from time import sleep
from geometry_msgs.msg import PoseStamped
import subprocess

class GoalExplorer(Node):

    def __init__(self):
        super().__init__('goal_explorer')

        self.create_action_client()
        self._map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.latest_map_data = None
        self.goal_in_progress = False
        self.goal_points = []
        self.goal_handle = None
        self.error_count = 0  # Đếm số lần lỗi liên tiếp
        self.error_threshold = 1  # Ngưỡng lỗi để khởi động lại server

        # Thử lấy vị trí robot sau khi khởi động
        self.get_robot_pose()

    def create_action_client(self):
        """
        Tạo hoặc tái tạo Action Client.
        """
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server to be available...')

    def get_robot_pose(self):
        """
        Lấy vị trí hiện tại của robot và lưu lại.
        """
        self.get_logger().info("Getting current robot pose...")
        self.robot_pose = PoseStamped()
        self.robot_pose.pose.position.x = 0.0
        self.robot_pose.pose.position.y = 0.0
        self.robot_pose.pose.orientation.w = 1.0
        self.get_logger().info(f"Robot position: x={self.robot_pose.pose.position.x}, y={self.robot_pose.pose.position.y}")

    def map_callback(self, msg):
        """
        Callback nhận bản đồ từ SLAM và cập nhật dữ liệu.
        """
        self.latest_map_data = msg
        self.get_logger().info("Received updated map.")
        
        if not self.goal_in_progress:
            self.find_and_send_goal()

    def find_and_send_goal(self):
        """
        Tìm điểm goal và gửi mục tiêu đến Nav2.
        """
        if not self.latest_map_data:
            self.get_logger().warn("Map data not available.")
            return

        self.goal_points = list(self.find_goal(self.latest_map_data))

        if not self.goal_points:
            self.get_logger().info("No goal detected.")
            return

        self.get_logger().info(f"Found {len(self.goal_points)} goal points.")
        self.send_next_goal()

    def find_goal(self, map_data):
        """
        Tìm các điểm goal từ bản đồ.
        """
        num_repeats_per_quadrant = 20
        quadrants = [1, 2, 3, 4]

        def generate_point_in_quadrant(quadrant):
            if quadrant == 1:
                goal_x = random.randint(0, 8)
                goal_y = random.randint(0, 8)
            elif quadrant == 2:
                goal_x = random.randint(-8, 0)
                goal_y = random.randint(0, 8)
            elif quadrant == 3:
                goal_x = random.randint(-8, 0)
                goal_y = random.randint(-8, 0)
            else:
                goal_x = random.randint(0, 8)
                goal_y = random.randint(-8, 0)
            return goal_x, goal_y

        for quadrant in range(1, 5):
            for _ in range(num_repeats_per_quadrant):
                goal_x, goal_y = generate_point_in_quadrant(quadrant)
                self.get_logger().info(f"Checking quadrant {quadrant} with point at x={goal_x}, y={goal_y}")
                yield goal_x, goal_y

    def is_valid_goal(self, x, y):
        """
        Kiểm tra tính khả thi của một điểm goal.
        """
        if not self.latest_map_data:
            return False

        map_data = self.latest_map_data
        map_width = map_data.info.width
        map_height = map_data.info.height
        cell_x = int((x - map_data.info.origin.position.x) / map_data.info.resolution)
        cell_y = int((y - map_data.info.origin.position.y) / map_data.info.resolution)

        if cell_x < 0 or cell_x >= map_width or cell_y < 0 or cell_y >= map_height:
            self.get_logger().info(f"Goal at x={x}, y={y} is out of map bounds.")
            return False

        index = cell_y * map_width + cell_x
        if map_data.data[index] > 50:
            self.get_logger().info(f"Goal at x={x}, y={y} is blocked by obstacle.")
            return False

        return True

    def send_next_goal(self):
        """
        Gửi mục tiêu tiếp theo trong danh sách goal.
        """
        if not self.goal_points:
            self.get_logger().info("No goal points left to send.")
            return

        goal_x, goal_y = self.goal_points.pop(0)

        if goal_x is not None and goal_y is not None and self.is_valid_goal(goal_x, goal_y):
            self.send_goal(goal_x, goal_y)
            sleep(1)  # Thêm độ trễ giữa các lần gửi mục tiêu
        else:
            self.get_logger().info(f"Goal at x={goal_x}, y={goal_y} is not valid. Skipping.")
            self.send_next_goal()

    def send_goal(self, x, y):
        """
        Gửi mục tiêu tới Nav2.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_in_progress = True

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f'Sent goal: x={x}, y={y}')

    def goal_response_callback(self, future):
        """
        Callback khi nhận phản hồi từ Nav2.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by server. Incrementing error count.")
            self.error_count += 1
            self.goal_in_progress = False

            if self.error_count >= self.error_threshold:
                self.get_logger().info("Error threshold reached. Trying again after a delay.")
                sleep(2)  # Thử lại sau một khoảng thời gian
                self.send_next_goal()
                self.error_count = 0  # Đặt lại số lần lỗi
            else:
                self.get_logger().info("Retrying goal after a short delay.")
                sleep(1)  # Thử lại sau một thời gian ngắn
                self.send_next_goal()
            return

        self.get_logger().info("Goal accepted by server.")
        self.error_count = 0  # Đặt lại bộ đếm khi thành công
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Callback khi Nav2 hoàn thành điểm đến.
        """
        status = future.result().status
        if status == 4:
            self.get_logger().info("Goal reached!")
        else:
            self.get_logger().warning("Failed to reach goal. Retrying after a short delay.")
        self.goal_in_progress = False
        self.send_next_goal()

    def restart_controller_server(self):
        """
        Tái khởi động controller_server nếu gặp lỗi quá số lần cho phép.
        """
        self.get_logger().info("Restarting controller server due to consecutive errors...")
        subprocess.Popen(["ros2", "lifecycle", "set", "/controller_server", "configure"])
        sleep(1)
        subprocess.Popen(["ros2", "lifecycle", "set", "/controller_server", "activate"])

def main(args=None):
    rclpy.init(args=args)
    goal_explorer = GoalExplorer()
    rclpy.spin(goal_explorer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
