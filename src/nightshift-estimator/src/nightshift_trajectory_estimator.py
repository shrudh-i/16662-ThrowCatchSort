import numpy as np
import matplotlib.pyplot as plt
import rospy
from frankapy import FrankaArm
from autolab_core import RigidTransform
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

VERBOSE = True

class EstimatorNode():

    @classmethod
    def __init__(self) -> None:
      # Input:  Self
      # Output: None
      # ---
      print('Initializing Estimation Node...')
      self.fa = FrankaArm()
      self.points = np.array([])
      self.times = np.array([])
      self.reference_time = 0

      self.max_x = 2.0
      self.min_x = 0.7

      self.last_xy = None
      self.pub_threshold = 0.02 # Only update publisher if new intersect is different enough

      self.intersect_pos_msg = PointStamped()

      self.sub = rospy.Subscriber("/ball_detections", PointStamped, self.update_estimator)
      self.pub = rospy.Publisher("/intersect_position", PointStamped,queue_size=2)
      #self.traj_pub  = rospy.Publisher("/trajectory_marker", Marker, queue_size=1)

    @classmethod
    def reset(self) -> None:
        print("Reset Estimator Node...")
        self.points = np.array([])
        self.times = np.array([])
        self.last_xy = None
        self.reference_time = 0

    @classmethod
    def update_estimator(self, msg):
        # Input:    self node
        #           msg from camera
        # Output:   No explicit output
        #           Goal Position published
        # ---

        the_point = [msg.point.x, msg.point.y, msg.point.z]
        curr_translation = self.fa.get_pose().translation

        # Populate Values first time around if empty
        if self.points.size == 0:
            self.points = np.array([the_point])
            self.reference_time = msg.header.stamp.tosec()
        # Otherwise add to array
        else:
            self.points = np.append(self.points,[the_point], axis=0)
        
        self.times = np.append(self.times, np.array(msg.header.stamp.to_sec() - self.reference_time))
        
        if len(self.points) < 10:
            return
        
        coeffs = self.FitData(self.GetRecentData())

        # TODO: Change goal position
        new_goal = self.EstimateLanding(coeffs, curr_translation)

        self.intersect_pos_msg.header = msg.header
        self.intersect_pos_msg.point.x = new_goal[1]
        self.intersect_pos_msg.point.y = new_goal[2]
        self.intersect_pos_msg.point.z = new_goal[3]
        self.pub.publish(self.intersect_pos_msg)


    @classmethod
    def GetRecentData(self, back_len = 10):
        data_length = len(self.points)
        start_idx = data_length - back_len
        x = self.points[start_idx:, 0]
        y = self.points[start_idx:, 1]
        z = self.points[start_idx:, 2]
        times = self.times[start_idx:]
        return times, x, y, z

    @classmethod
    def FitData(times, x, y, z):
        # Input:    A set of points containing time and XYZ coords
        # Output:   Coefficients X,Y,Z plots
        # ---

        # X and Y coeffs should associate with linear vel
        coeff_x = np.polynomial.polynomial.polyfit(times, x, deg=1)
        coeff_y = np.polynomial.polynomial.polyfit(times, y, deg=1)
        # Z coeffs for t^2 should match (1/2) * g
        coeff_z = np.polynomial.polynomial.polyfit(times, z, deg=2)

        # The coefficients are given in reverse order, thus the flip
        return [np.flip(coeff_x), np.flip(coeff_y), np.flip(coeff_z)]


    @classmethod
    def EstimateLanding(coeffs, goal):
        # Input:    Coefficients for the X,Y,Z plots
        #           Goal position [X,Y,Z]
        # Output:   Time and Position of Landing
        # ---

        # coeffs[0]: X, coeffs[1]: Y, coeffs[2]: Z
        
        min_dist = np.Inf
        del_t = 0.4
        lookahead_time = 20 # seconds

        func_x, func_y, func_z = np.poly1d(coeffs[0]),  np.poly1d(coeffs[1]),  np.poly1d(coeffs[2])
        distance =[]
        for i in (rospy.get_time(),lookahead_time,del_t):
            x_i, y_i, z_i = func_x(i), func_y(i), func_z(i)
            # print("\n", x_i, y_i, z_i)

            # Euclidean distance from the current position to the desired position
            dist = np.sqrt((goal[0] - x_i)**2 + (goal[1]- y_i)**2 + (goal[2] - z_i)**2)
            distance.append(dist)
            if dist<min_dist:
                min_dist = dist
                x_d, y_d, z_d = x_i, y_i, z_i
                toi = i
            elif dist > min_dist:
                return [toi, x_d, y_d, z_d]
        

if __name__ == "__main__":
    pass