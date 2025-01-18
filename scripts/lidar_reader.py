import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 1024)
        self._lidar = LaserScan()
        self.output_lidar_data = []  # Dùng một danh sách duy nhất để chứa góc và khoảng cách

    @property
    def lidar(self):
        return self._lidar

    @lidar.setter
    def lidar(self, value):
        self._lidar = value

    def cb_lidar(self, msg: LaserScan):
        self.lidar = msg
        self.hien_thi_lidar()
        self.hien_thi_vat_can()

    def hien_thi_lidar(self):
        self.output_lidar_data.clear()  # Làm sạch dữ liệu trước khi cập nhật

        for i, distance in enumerate(self.lidar.ranges):
            if distance != float('inf'):  # Kiểm tra khoảng cách hợp lệ
                angle = self.lidar.angle_min + i * self.lidar.angle_increment
                angle_deg = math.degrees(angle)
                # Lưu cả góc và khoảng cách trong một tuple
                self.output_lidar_data.append((angle_deg, distance))

def hien_thi_vat_can(self):
    if len(self.output_lidar_data) < 2:
        return  

    clusters = []  # Danh sách các cụm vật cản
    current_cluster = []  # Cụm hiện tại
    nguong_gop_can = 2  # Ngưỡng góc gộp, đơn vị độ
    
    # Bắt đầu từ điểm đầu tiên
    for i in range(len(self.output_lidar_data) - 1):
        current_cluster.append(self.output_lidar_data[i])  # Thêm điểm vào cụm

        # Kiểm tra góc giữa các điểm có vượt ngưỡng không
        angle_diff = abs(self.output_lidar_data[i+1][0] - self.output_lidar_data[i][0])  # Chỉ tính góc
        if angle_diff > nguong_gop_can:
            if current_cluster:
                clusters.append(current_cluster)  # Thêm cụm vào danh sách
                current_cluster = []  # Khởi tạo lại cụm mới
    
    # Thêm cụm cuối cùng nếu có
    if current_cluster:
        clusters.append(current_cluster)

    # In thông tin về các vật cản
    for cluster in clusters:
        start_angle = cluster[0][0]  # Góc bắt đầu của cụm
        end_angle = cluster[-1][0]   # Góc kết thúc của cụm
        min_distance = min([point[1] for point in cluster])  # Khoảng cách gần nhất trong cụm

        self.get_logger().info(
            f"Phát hiện vật cản: Góc {start_angle:.2f}° -> {end_angle:.2f}°, "
            f"khoảng cách gần nhất: {min_distance:.2f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReader()
    rclpy.spin(lidar_reader)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
