import rospy
import tf
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import TransformStamped

class RobotTrajectoryPlotter:
    def __init__(self):
        rospy.init_node('robot_trajectory_plotter', anonymous=True)
        self.tf_listener = tf.TransformListener()
        self.times = []
        self.x_positions = []
        self.y_positions = []
        self.rate = rospy.Rate(10)  # 10 Hz
        self.start_time = rospy.get_time()

    def get_transform(self):
        try:
            # Lấy transform từ map đến base_link
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Không thể lấy transform từ map đến base_link")
            return None

    def collect_data(self):
        while not rospy.is_shutdown():
            trans = self.get_transform()
            if trans:
                current_time = rospy.get_time() - self.start_time
                self.times.append(current_time)
                self.x_positions.append(trans[0])
                self.y_positions.append(trans[1])
            self.rate.sleep()

    def plot_and_save(self):
        # Tạo figure với 3 subplot
        plt.figure(figsize=(15, 5))

        # Biểu đồ 1: x theo thời gian
        plt.subplot(1, 3, 1)
        plt.plot(self.times, self.x_positions, 'b-', label='Vị trí x')
        plt.xlabel('Thời gian (s)')
        plt.ylabel('Vị trí x (m)')
        plt.title('Vị trí x theo thời gian')
        plt.grid(True)
        plt.legend()

        # Biểu đồ 2: y theo thời gian
        plt.subplot(1, 3, 2)
        plt.plot(self.times, self.y_positions, 'r-', label='Vị trí y')
        plt.xlabel('Thời gian (s)')
        plt.ylabel('Vị trí y (m)')
        plt.title('Vị trí y theo thời gian')
        plt.grid(True)
        plt.legend()

        # Biểu đồ 3: Quỹ đạo x-y
        plt.subplot(1, 3, 3)
        plt.plot(self.x_positions, self.y_positions, 'g-', label='Quỹ đạo')
        plt.xlabel('Vị trí x (m)')
        plt.ylabel('Vị trí y (m)')
        plt.title('Quỹ đạo robot')
        plt.grid(True)
        plt.legend()
        plt.axis('equal')  # Đảm bảo tỷ lệ trục x và y bằng nhau

        # Lưu biểu đồ
        plt.tight_layout()
        plt.savefig('robot_trajectory.png')
        rospy.loginfo("Đã lưu biểu đồ vào robot_trajectory.png")

if __name__ == '__main__':
    try:
        plotter = RobotTrajectoryPlotter()
        # Thu thập dữ liệu trong 30 giây
        end_time = rospy.get_time() + 30
        while rospy.get_time() < end_time and not rospy.is_shutdown():
            plotter.collect_data()
        # Vẽ và lưu biểu đồ
        plotter.plot_and_save()
    except rospy.ROSInterruptException:
        pass