#!/usr/bin/env python3

import time
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage

VERBOSE = True

from franka_interface_msgs.msg import SensorDataGroup

class PlannerNode:
    @classmethod
    def __init__(self):
        self.fa = FrankaArm()
        ##TODO: Modify this with our end-effector 
        # self.fa.set_tool_delta_pose(self.NIGHTSHIFT_TOOL_POSE)

        self.reset()
    
    @classmethod
    def reset(self):
        '''
        Reset the Franka Arm
        '''

        if VERBOSE: 
            rospy.loginfo("Resetting the Franka Arm")

        self.fa.reset_joints()
        self.fa.stop_skill() # stops the current skill 
        self.fa.wait_for_skill() # waits for the current skill to be stopped

        self.dynamic_setup()

        if VERBOSE:
            rospy.loginfo("Reset Complete")

    
    @classmethod
    def dynamic_setup(self):
        '''
        #TODO: Function description
        '''

        if VERBOSE:
            rospy.loginfo("Setting up dynamic params")

        # init the goal 
        # self.goal_pose = None

        # retrieve goal position:
        self.sub = rospy.Subscriber(
            #TODO: Confirm on the topic name
            "",
            PointStamped,
            self.set_goal()
        )

        self.pub = rospy.Publisher(
            FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
            SensorDataGroup,
            queue_size=1000
        )
        print(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)

        #TODO: Check if this is required
        #TODO: Confirm the speed at which we want the arm to move (duration)
        self.dt = 0.01
        # self.rate = rospy.Rate(1 / self.dt)

        self.disp = 0.2
        rospy.Time.now().to_time() # retrieve current time
        self.id = 0 # further used when moving to catch point

        self.last_validpose_time = time.time()

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.moveToCatch)
        self.timeout = 5

        current = self.fa.get_pose()
        print("Initial pose: ", current.translation) 
        self.fa.goto_pose(current, duration=self.dt, dynamic=True, buffer_time=500, block=False)

        if VERBOSE:
            rospy.loginfo("Dynamic params setup")


    @classmethod
    def set_goal(self, goal_pose):
        '''
        Callback to store the subscribed goal pose 
        '''

        self.goal_pose = goal_pose