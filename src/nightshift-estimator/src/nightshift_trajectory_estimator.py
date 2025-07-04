#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospy
# from frankapy import FrankaArm
from autolab_core import RigidTransform
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker

VERBOSE = False

class EstimatorNode():

    @classmethod
    def __init__(self) -> None:
      # Input:  Self
      # Output: None
      # ---
      if VERBOSE:
        print('Initializing Estimation Node...')

    #   self.fa = FrankaArm(with_gripper=False)
      self.points = np.array([])
      self.times = np.array([])
      self.reference_time = 0

      self.max_lookback = 5

      self.max_x = 2.0
      self.min_x = 0.7

      self.last_xy = None
      self.pub_threshold = 0.02 # Only update publisher if new intersect is different enough

      self.intersect_pos_msg = PointStamped()

      self.sub = rospy.Subscriber("/ball_detections", PointStamped, self.update_estimator)
      self.pub = rospy.Publisher("/intersect_position", PointStamped,queue_size=2)
      self.traj_pub  = rospy.Publisher("/trajectory_marker", Marker, queue_size=1)
      if VERBOSE:
          print('Estimator Node Initialized')

    @classmethod
    def reset(self) -> None:
        if VERBOSE:
            print("Reset Estimator Node...")
        self.points = np.array([])
        self.times = np.array([])
        self.last_xy = None
        self.reference_time = 0
        self.max_lookback = 5
        if VERBOSE:
            print("Estimator Node Successfully Reset")

    @classmethod
    def update_estimator(self, msg):
        # Input:    self node
        #           msg from camera
        # Output:   No explicit output
        #           Goal Position published
        # ---

        the_point = [msg.point.x, msg.point.y, msg.point.z]
        # curr_translation = self.fa.get_pose().translation
        curr_translation = [3.07031348e-01, -2.66994994e-04,  4.85655589e-01]

        # Populate Values first time around if empty
        if self.points.size == 0:
            self.points = np.array([the_point])
            self.reference_time = msg.header.stamp.to_sec()
        # Otherwise add to array
        else:
            self.points = np.append(self.points,[the_point], axis=0)
        
        self.times = np.append(self.times, np.array(msg.header.stamp.to_sec() - self.reference_time))

        # Resize points array to size of self.max_lookback
        if len(self.points) > self.max_lookback:
            self.points = np.delete(self.points,0,axis=0)
            self.times = np.delete(self.times, 0)
            # if VERBOSE:
            #     print("Points array of length: ", len(self.points))
        
        if len(self.points) < self.max_lookback:
            if VERBOSE:
                print("Not enough data")
            return
        
        r_times, r_x, r_y, r_z = self.GetRecentData()
        coeffs = self.FitData(r_times, r_x, r_y, r_z)

        # TODO: Change goal position
        new_goal = self.EstimateLanding(coeffs, curr_translation, msg)

        if new_goal != None:

            if VERBOSE: 
                print("new goal: ", new_goal)
            self.intersect_pos_msg.header = msg.header
            self.intersect_pos_msg.point.x = new_goal[1]
            self.intersect_pos_msg.point.y = new_goal[2]
            self.intersect_pos_msg.point.z = new_goal[3]
            self.pub.publish(self.intersect_pos_msg)
        else:
            if VERBOSE:
                print("New Goal Not Valid")
                print("unvalid goal: ", new_goal)


    @classmethod
    def GetRecentData(self, back_len = 10):
        back_len = min(back_len, self.max_lookback)
        data_length = len(self.points)
        start_idx = data_length - back_len
        x = self.points[start_idx:, 0]
        y = self.points[start_idx:, 1]
        z = self.points[start_idx:, 2]
        times = self.times[start_idx:]
        return times, x, y, z

    @classmethod
    def FitData(self, times, x, y, z):
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
    def EstimateLanding(self, coeffs, goal, msg):
        # Input:    Coefficients for the X,Y,Z plots
        #           Goal position [X,Y,Z]
        # Output:   Time and Position of Landing
        # ---

        # coeffs[0]: X, coeffs[1]: Y, coeffs[2]: Z
        
        min_dist = np.Inf
        del_t = 0.2
        lookahead_time = 2 # seconds

        func_x, func_y, func_z = np.poly1d(coeffs[0]),  np.poly1d(coeffs[1]),  np.poly1d(coeffs[2])

        # distance =[]
        
        r_times, _, _, _ = self.GetRecentData()
        # print(len(r_times))
        # print(np.linspace(r_times[len(r_times)-1],lookahead_time,del_t))
        traj_points = []

        for i in np.arange(r_times[len(r_times)-1],r_times[len(r_times)-1]+lookahead_time,del_t):
            if VERBOSE:
                # print("entering loop...")
                print("iteration i:", i)
            x_i, y_i, z_i = func_x(i), func_y(i), func_z(i)
            
            # Euclidean distance from the current position to the desired position
            dist = np.sqrt((goal[0] - x_i)**2 + (goal[1]- y_i)**2 + (goal[2] - z_i)**2)
            # distance.append(dist)
            if dist<min_dist:
                min_dist = dist
                x_d, y_d, z_d = x_i, y_i, z_i
                traj_points.append(Point())
                traj_points[-1].x = x_d
                traj_points[-1].y = y_d
                traj_points[-1].z = z_d
                toi = i
                # toi = i - r_times[len(r_times)-1]
            elif dist > min_dist:
                #print("goal - x, y, z: ", x_d, y_d, z_d)

                traj_marker = Marker()
                traj_marker.type = traj_marker.POINTS
                traj_marker.header = msg.header
                traj_marker.points = traj_points
                traj_marker.action = traj_marker.ADD
                traj_marker.scale.x = 0.05
                traj_marker.scale.y = 0.05
                traj_marker.scale.z = 0.05
                traj_marker.color.r = 1.0
                traj_marker.color.a = 1.0
                traj_marker.pose.orientation.w = 1.0
                self.traj_pub.publish(traj_marker)
                return [toi, x_d, y_d, z_d]
        

if __name__ == "__main__":
    rospy.init_node('nightshift-estimator')
    # rospy.spin()
    # fa = FrankaArm() 
    EstimatorNode()
    rospy.spin()