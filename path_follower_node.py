#!/usr/bin/env python3

import sys
import os

# Menambahkan folder utils ke sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'utils'))

# Sekarang Anda bisa mengimpor file dari folder 'utils'
import rospy
from marker_publisher import MarkerPublisher
from basic_pid import BasicPID
from fuzzy_pid import FuzzyPID
from boat_tf import BoatTF
from pid_mrac import PIDMRACMITRule
from utility import get_angle, rad_to_deg, distance_line_to_point
from sensor_msgs.msg import NavSatFix  # Impor NavSatFix
from nav_msgs.msg import Path  # Impor Path dari nav_msgs.msg

class PathFollower:
    def __init__(self) -> None:
        rospy.loginfo("Initializing PathFollower Node")

        self.objects = []

        # Ganti FuzzyPID dengan PID-MRAC menggunakan MIT Rule
        self.pid_angle = PIDMRACMITRule()  # Menggunakan MIT Rule untuk kontrol yaw
        self.pid_distance = PIDMRACMITRule()  # Menggunakan MIT Rule untuk kontrol surge

        # Inisialisasi PID gains awal (untuk MIT Rule, nilai ini akan disesuaikan otomatis)
        self.pid_angle.p = [0.02, 0.03]  # Koefisien awal Kp, Ki, Kd untuk yaw
        self.pid_distance.p = [0.4, 0.6]  # Koefisien awal Kp, Ki, Kd untuk surge
        self.pid_distance.upper_bound = 1.0
        self.pid_distance.lower_bound = -1.0

        # Error variables for adaptive PID
        self.error_angle = 0.0
        self.error_distance = 0.0
        self.path_angle_limit = 60
        self.path_keep_distance = 2.0

        self.main_path = Path()  # Inisialisasi Path
        self.current_position = NavSatFix()  # Inisialisasi NavSatFix

        self.path_idx = 1
        self.path_idx_before = 0

        rospy.loginfo("MarkerPublisher and BoatTF initialized")
        self.marker_publisher = MarkerPublisher()
        self.boat_tf = BoatTF()

        # Timer untuk update secara berkala
        rospy.Timer(rospy.Duration(1.0), self.update)  # Update setiap 1 detik

    def update(self, event):
        rospy.loginfo("Updating PathFollower")

        self.boat_tf.listen_TF()
        if not self.boat_tf.is_tf_available:
            rospy.logwarn("Transform data not available")
            return 0

        # Check if arrived to last waypoint
        if self.is_arrived_last_WP():
            rospy.loginfo("ARRIVED TO LAST WP")
            return 0

        # If no more waypoints, exit
        if len(self.main_path.poses) < self.path_idx + 1:
            rospy.loginfo("NO WAYPOINT LEFT")
            return 0

        if self.is_arrived_path():
            rospy.loginfo(f"ARRIVED {self.path_idx}")
            self.path_idx += 1

        # Get angle and distance error
        angle_path = get_angle(self.main_path.poses[self.path_idx - 1].pose.position,
                               self.main_path.poses[self.path_idx].pose.position)
        path_error_angle = rad_to_deg(angle_path - self.boat_tf.yaw)

        # Normalize error angle
        if path_error_angle < -180:
            path_error_angle += 360
        elif path_error_angle > 180:
            path_error_angle -= 360

        path_error_dist = distance_line_to_point(self.main_path.poses[self.path_idx - 1].pose.position, 
                                                 self.main_path.poses[self.path_idx].pose.position, self.boat_tf.pos)

        # Adaptive PID for path following using MIT Rule
        # Update PID with MIT Rule (error angle and distance)
        path_control_angle_out = self.pid_angle.update(path_error_angle, self.main_path.poses[self.path_idx].pose.position, 1.0)
        path_control_distance_out = self.pid_distance.update(path_error_dist, self.main_path.poses[self.path_idx].pose.position, 1.0)

        # Sum the results of angle and distance control
        result = path_control_angle_out + path_control_distance_out
        return result

    def is_arrived_path(self):
        if len(self.main_path.poses)-1 < self.path_idx:
            return True
        controlled_path_point = self.main_path.poses[self.path_idx].pose.position
        controlled_path_point_before = self.main_path.poses[self.path_idx - 1].pose.position
        
        angle = rad_to_deg(get_angle(self.boat_tf.pos, controlled_path_point) - get_angle(controlled_path_point_before, controlled_path_point))
        
        delta_x = self.boat_tf.pos.x - controlled_path_point.x
        delta_y = self.boat_tf.pos.y - controlled_path_point.y

        if np.abs(angle) >= 180:
            angle = 360 - np.abs(angle)
        return not(abs(angle) < self.path_angle_limit and pow(delta_x, 2) + pow(delta_y, 2) > pow(self.path_keep_distance,2))
    
    def is_arrived_last_WP(self):
        if self.path_idx >= len(self.main_path.poses)-1:
            if self.is_arrived_path():
                return True
        return False
    
    def get_path_angle(self):
        if len(self.main_path.poses) <= 1:
            rospy.logwarn("WARNING: Not enough waypoints")
            return

        angle_path = get_angle(self.main_path.poses[self.path_idx - 1].pose.position,
                                        self.main_path.poses[self.path_idx].pose.position)
        return angle_path
    
    def set_path(self, path : Path):
        self.main_path = path
        self.path_idx = 1
        self.path_idx_before = 0
        rospy.loginfo("Path has been set")

# Jalankan ROS node
if __name__ == '__main__':
    try:
        rospy.init_node('path_follower_node', anonymous=True)
        path_follower = PathFollower()
        rospy.spin()  # Menjaga agar node tetap berjalan
    except rospy.ROSInterruptException:
        pass
