import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class TwoWheeledRobot(Node):
    def __init__(self):
        super().__init__('two_wheeled_robot')

        # Publisher cho các topic điều khiển
        self.pubCmd = self.create_publisher(Twist, '/cmd_vel', 10)  # Topic cmd_vel mà diff_drive đang nghe
        self.pubPosCtrl = self.create_publisher(Bool, '/posctrl', 10)
        self.pubVelMode = self.create_publisher(Bool, '/carvel_mode', 10)
        
        # Biến trạng thái robot
        self.isFlying = True  # Giả sử robot đang di chuyển
        self.isPosctrl = False
        self.isVelMode = False

    def move(self, linear_x: float, angular_z: float):
        """
        Điều khiển robot di chuyển bằng cách thay đổi vận tốc tuyến tính và vận tốc góc.
        :param linear_x: Vận tốc tuyến tính trên trục x (m/s)
        :param angular_z: Vận tốc góc trên trục z (rad/s)
        """
        if not self.isFlying:
            self.get_logger().info("Robot is not moving!")
            return False

        # Tạo thông điệp Twist để gửi lệnh di chuyển
        twist_msg = Twist()
        twist_msg.linear.x = linear_x  # Di chuyển tiến/lùi
        twist_msg.angular.z = angular_z  # Quay trái/phải
        
        # Gửi lệnh di chuyển tới /cmd_vel
        self.pubCmd.publish(twist_msg)
        self.get_logger().info(f"Moving with linear_x: {linear_x}, angular_z: {angular_z}")
        return True

    def hover(self):
        """
        Giữ robot ở vị trí hiện tại (hover).
        :return: True nếu lệnh được gửi thành công, False nếu robot không đang di chuyển
        """
        if not self.isFlying:
            self.get_logger().info("Robot is not moving!")
            return False
        
        # Gửi lệnh di chuyển với vận tốc bằng 0 (giữ vị trí)
        twist_msg = Twist()
        self.pubCmd.publish(twist_msg)
        return True

    def posCtrl(self, on: bool):
        """
        Bật/tắt chế độ điều khiển vị trí.
        :param on: True để bật điều khiển vị trí, False để tắt
        """
        self.isPosctrl = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubPosCtrl.publish(bool_msg)
        return True

    def velMode(self, on: bool):
        """
        Bật/tắt chế độ điều khiển vận tốc.
        :param on: True để bật chế độ vận tốc, False để tắt
        :return: True nếu lệnh được gửi thành công, False nếu robot không đang di chuyển
        """
        if not self.isFlying:
            self.get_logger().info("Robot is not moving!")
            return False

        self.isVelMode = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubVelMode.publish(bool_msg)
        return True


def main(args=None):
    rclpy.init(args=args)

    robot = TwoWheeledRobot()

    # Kiểm tra xem robot có thể di chuyển không
    robot.move(0.5, 0.1)  # Di chuyển robot với vận tốc 0.5 m/s và quay 0.1 rad/s
    
    # Đợi một chút rồi dừng robot
    rclpy.spin_once(robot)  # Chạy 1 lần để robot thực hiện di chuyển
    rclpy.sleep(5)  # Đợi 5 giây

    # Giữ robot ở vị trí hiện tại
    robot.hover() 
    
    # Tắt chế độ điều khiển vị trí và vận tốc
    robot.posCtrl(False)
    robot.velMode(False)
    
    rclpy.spin(robot)  # Giữ node chạy

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
