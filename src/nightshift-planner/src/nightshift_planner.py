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

    # End-Effector pose
    NIGHTSHIFT_TOOL_POSE = RigidTransform(
            # 8cm going down; 16cm going out from the end-effector base 
            translation=np.array([0.16, 0, 0.08]),
            rotation=np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0,  0, 1]]),
            from_frame="franka_tool",
            to_frame="franka_tool_base"
        )

    #TODO: verify if this is a good pose from which the arm can start moving from
    START_POSE = RigidTransform(
            translation=np.array([0.56, 0, 0.4]),
            rotation=np.array([[7.07185286e-01, -7.07028188e-01, -3.36139057e-04],
                                [-7.07028245e-01, -7.07185062e-01, -5.90835436e-04],
                                [1.80024788e-04,  6.55489934e-04, -9.99999769e-01]]),
            from_frame="franka_tool",
            to_frame="world"
        )

    #TODO: verify if this bounding box is what we want to go with
    BOX_CORNER_MIN = np.array([0.46, -0.3, 0.2])
    BOX_CORNER_MAX = np.array([0.66, 0.3, 0.7])


    @classmethod
    def __init__(self):
        print("FRANKA ARM IS GETTING INITIALIZED")
        self.fa = FrankaArm(with_gripper=False) 

        self.fa.set_tool_delta_pose(self.NIGHTSHIFT_TOOL_POSE)
        

        self.is_resetting = True
        self.reset()
    
    @classmethod
    def reset(self):
        '''
        Reset the Franka Arm
        '''

        if VERBOSE: 
            rospy.loginfo("Resetting the Franka Arm")
        self.is_resetting = True
        self.fa.reset_joints()

        #TODO: verify if we want the arm to start form a static pose
        self.fa.goto_pose(self.START_POSE, duration=2)

        self.fa.stop_skill() # stops the current skill 
        self.fa.wait_for_skill() # waits for the current skill to be stopped

        self.dynamic_setup()
        
        self.is_resetting = False
        if VERBOSE:
            rospy.loginfo("Reset Complete")

    
    @classmethod
    def dynamic_setup(self):
        '''
        Setting up dynamic params before catching
        '''

        if VERBOSE:
            rospy.loginfo("Setting up dynamic params")

        # init the goal_pose 
        self.goal_msg = None

        # retrieve goal_pose position:
        self.sub = rospy.Subscriber(
            #TODO: Confirm on the topic name
            "/intersect_position",
            PointStamped,
            self.set_goal
        )

        self.pub = rospy.Publisher(
            FC.DEFAULT_SENSOR_PUBLISHER_TOPIC,
            SensorDataGroup,
            queue_size=1000
        )
        if VERBOSE:
            print(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC)

        #TODO: Confirm the speed at which we want the arm to move (duration)
        self.dt = 0.1

        #TODO: Check if this is required
        self.rate = rospy.Rate(1 / self.dt)

        self.disp = 0.25
        self.init_time = rospy.Time.now().to_time() # retrieve current time
        self.id = 0 # further used when moving to catch point
        self.last_validpose_time = time.time()

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.catch)
        self.timeout = 2

        current = self.fa.get_pose()
        
        if VERBOSE:
            print("Initial pose: ", current.translation) 
            print("Initial rotatiom:", current.rotation)
            # pass

        print("first move goto_pose")
        # self.fa.goto_pose(current, duration=self.dt, dynamic=True, buffer_time=500, block=False)
        # self.fa.wait_for_skill()
        current = self.fa.get_pose()
        self.fa.goto_pose(current, duration=self.dt, dynamic=True, buffer_time=500, block=False)
        self.fa.wait_for_skill()

        if VERBOSE:
            rospy.loginfo("Dynamic params set")


    @classmethod
    def set_goal(self, goal_msg):
        '''
        Callback to store the subscribed goal_pose pose 
        '''
        self.goal_msg = goal_msg
        # print(self.goal_msg)

    @classmethod
    def catch(self, event):
        '''
        Moving the Franka Arm to the catch point
        '''

        if self.is_resetting:
            return

        current_pose = self.fa.get_pose()
        if np.any(np.isnan(current_pose.translation)) or np.any(np.isnan(current_pose.quaternion)):
            if VERBOSE:
                print("Current pose has Nan")

        if self.goal_msg is None:
            goal_pose = current_pose
            # goal_pose = event
        else:
            goal_pose = RigidTransform(
                translation = np.array([
                    self.goal_msg.point.x,
                    self.goal_msg.point.y,
                    self.goal_msg.point.z,
                ]),
                from_frame=current_pose.from_frame,
                to_frame=current_pose.to_frame,
            )

        # store a copy of the current arm pose
        intermediate = current_pose

        # moving by a delta 
        displacement = goal_pose.translation - current_pose.translation
        displ_norm = np.linalg.norm(displacement)

        #TODO: verify if this heuristic holds
        if np.abs(displ_norm) > 1e-3:
            displacement = min(displ_norm, self.disp) * (displacement/displ_norm)
        
        intermediate.translation = current_pose.translation + displacement
        
        timestamp = rospy.Time.now().to_time() - self.init_time
        # print(displacement)

        if not self.is_valid_pose(intermediate):
            if VERBOSE:
                print("Invalid Pose:", intermediate.translation)
                # pass
            intermediate = current_pose
        else:
            self.last_validpose_time = time.time()

        # account for the tool offset
        if np.any(np.isnan(intermediate.translation)):
            intermediate = current_pose
        
        intermediate = intermediate * self.NIGHTSHIFT_TOOL_POSE.inverse()

        traj_gen_msg = PosePositionSensorMessage(
            id=self.id,
            timestamp=timestamp,
            # timestamp=15,
            position=intermediate.translation,
            #TODO: Check if the START_POSE is the same name being maintained
            quaternion=self.START_POSE.quaternion,
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_msg, SensorDataMessageType.POSE_POSITION
            )
        )

        if np.any(np.isnan(intermediate.translation)):
            if VERBOSE:
                print("Nan in message")
        self.pub.publish(ros_msg)
        self.id += 1
        
        self.fa.wait_for_skill()
        self.fa.stop_skill()

        print('Sorting starts')
        print('Start position')
        self.fa.goto_joints( [0.13354869, -0.09719834, -0.17910186, -2.87180187,  0.0552314,   2.83424865, 0.70680245])

        print('Left position')
        self.fa.goto_joints([ 0.00350013,  0.04199704, -0.67248926, -2.47509156,  1.30884392,  1.13389734, 0.3047798 ])
        time.sleep(2)

        print('Start position')
        self.fa.goto_joints( [0.13354869, -0.09719834, -0.17910186, -2.87180187,  0.0552314,   2.83424865, 0.70680245])

        if VERBOSE:
            print("TIME DIFFERENCE: ",time.time() - self.last_validpose_time)
            # pass

        if (time.time() - self.last_validpose_time > self.timeout):
            self.is_resetting = True
            if VERBOSE:
                print('Terminating...')
            self.terminate_dynamic()
            if VERBOSE:
                print("Terminated")
            self.reset()
        
        ###################TEMP####################

        # print("Current error: ", np.linalg.norm(curr_pose.translation - goal_pose.translation))
        # print("Interpol pose: ", interpol_pose.translation)
        # print("Delta: ", delta)

    @classmethod
    def is_valid_pose(self, pose):
        '''
        '''
        return np.all(pose.translation <= self.BOX_CORNER_MAX) and np.all(pose.translation >= self.BOX_CORNER_MIN)

    @classmethod
    def terminate_dynamic(self):
        '''
        '''
        terminate_msg = ShouldTerminateSensorMessage(
            timestamp=rospy.Time.now().to_time() - self.init_time, 
            should_terminate=True
        )

        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                terminate_msg, 
                SensorDataMessageType.SHOULD_TERMINATE
            )
        )
        self.pub.publish(ros_msg)
        self.fa.wait_for_skill()

if __name__ == "__main__":
    
    # rospy.init_node('nightshift-planner')
    PlannerNode()
    # while not rospy.is_shutdown():
    rospy.spin()
    # rospy.spin()
    # fa = FrankaArm(with_gripper=False)
    # print("In main")
    # pn = PlannerNode()
    # print(fa.get_pose())
    # print(fa.get_pose().translation)

    # time.sleep(5)
    # print(fa.get_tool_base_pose())
    # # Go to start position
    # fa.set_tool_delta_pose(pn.NIGHTSHIFT_TOOL_POSE)
    # print(fa.set_tool_delta_pose(pn.NIGHTSHIFT_TOOL_POSE))
    

    # # Create a displacement w.r.t the tool and create a goal pose
    # curr_pose = fa.get_pose()
    # T_delta = RigidTransform(
    #     translation=np.array([0.04, -0.1032, 0.0]),
    #     from_frame=curr_pose.to_frame,
    #     to_frame=curr_pose.to_frame,
    # )
    
    # goal_pose = T_delta * curr_pose
    # print("Goal Pose:", goal_pose)

    # # pn.catch(fa, goal_pose)
    # # pn.catch()
    # pn.catch(goal_pose)