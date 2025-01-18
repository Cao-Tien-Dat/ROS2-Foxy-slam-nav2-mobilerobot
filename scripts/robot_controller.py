import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class Control(Node):
    def __init__(self):
        super().__init__('controller')
        self.pubCmd = self.create_publisher(Twist, '/cmd_vel', 1024)
        self.isDriving = True

        # Subscribers
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 1024)
        self._lidar = LaserScan()

    @property
    def lidar(self):
        return self._lidar

    @lidar.setter
    def lidar(self, value):
        self._lidar = value

    def cb_lidar(self, msg: LaserScan):
        self.lidar = msg

        self.kc_rad() # msg- > self.lidar -> kc_rad() -> (khoang_cach, goc).

    def kc_rad(self):
        # (khoang cách, goc)    angle_increment = goc giua 2 tin hieu lidar lien tiep
        for i, distance in enumerate(self.lidar.ranges):
            angle = self.lidar.angle_min + i * self.lidar.angle_increment 
            self.get_logger().info(f"Khoảng cách: {distance:.2f} m, Góc: {math.degrees(angle):.2f} độ")


    def move(self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0):
        if not self.isDriving:
            self.get_logger().info("xe_dung")
            return False
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        self.pubCmd.publish(twist_msg)
        self.get_logger().info(f"linear_x: {linear_x}, linear_y: {linear_y}, angular_z: {angular_z}")
        return True

    def stop(self):
        self.get_logger().info("Dừng robot....")
        twist_msg = Twist()
        self.pubCmd.publish(twist_msg)

    # Các hướng di chuyển gửi tới phương thức move
    def diThang(self):
        self.move(linear_x=0.5, linear_y=0.0, angular_z=0.0)

    def diLui(self):
        self.move(linear_x=-0.5, linear_y=0.0, angular_z=0.0)

    def reTrai(self):
        self.move(linear_x=0.0, linear_y=0.0, angular_z=0.5)

    def rePhai(self):
        self.move(linear_x=0.0, linear_y=0.0, angular_z=-0.5)

    def ThangTrai(self):
        self.move(linear_x=0.5, linear_y=0.5, angular_z=0.0)

    def thangPhai(self):
        self.move(linear_x=0.5, linear_y=-0.5, angular_z=0.0)

    def luiTrai(self):
        self.move(linear_x=-0.5, linear_y=0.5, angular_z=0.0)

    def luiPhai(self):
        self.move(linear_x=-0.5, linear_y=-0.5, angular_z=0.0)


def main(args=None):
    rclpy.init(args=args)
    robot = Control()
    # Vòng lặp ROS2
    rclpy.spin(robot)  # Sử dụng rclpy.spin để duy trì node đang hoạt động và xử lý các callback.

    rclpy.shutdown()


if __name__ == '__main__':
    main()
