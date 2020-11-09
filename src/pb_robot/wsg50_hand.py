import numpy
import pybullet as p
import pb_robot

import rospy
#import robot_comm.srv as srv

class WSG50Hand(pb_robot.body.Body):
    '''Set position commands for the panda hand. Have not yet included
    gripping with force.'''
    def __init__(self, bodyID=None, left_finger_name='base_joint_gripper_left', right_finger_name='base_joint_gripper_right'):
        '''Pull left and right fingers from robot's joint list'''
        if bodyID is None:
            urdf_file = 'models/wsg50_description/wsg_50.urdf'
            with pb_robot.helper.HideOutput():
                with pb_robot.utils.LockRenderer():
                    bodyID = pb_robot.utils.load_model(urdf_file, fixed_base=True)

        pb_robot.body.Body.__init__(self, bodyID)
        self.left_finger = self.joint_from_name(left_finger_name)
        self.right_finger = self.joint_from_name(right_finger_name)

    def Open(self):
        '''Open the fingers by setting their positions to the upper limit'''
        self.left_finger.set_joint_position(-0.055)
        self.right_finger.set_joint_position(0.055)

    def Close(self):
        '''Close the fingers by setting their positions to the inner limit'''
        self.left_finger.set_joint_position(-0.0027)
        self.right_finger.set_joint_position(0.0027)

    def MoveTo(self, left_distance, right_distance):
        '''Move the fingers uniformally such that 'distance' is the width
        between the two fingers. Therefore, each each finger will move 
        distance/2. 
        @param distance Desired distance between fingers'''
        # left: limit lower="-0.055" upper="-0.0027"
        # right: lower="0.0027" upper="0.055"

        if not (-0.055 <= left_distance <= -0.0027):
            raise IOError("Invalid distance request. The value must be between -0.055 and -0.0027")
        if not (0.0027 <= right_distance <= 0.055):
            raise IOError("Invalid distance request. The value must be between 0.027 and 0.055")

        self.left_finger.set_joint_position(left_distance)
        self.right_finger.set_joint_position(right_distance)

    def GetJointPositions(self):
        '''Get the joint poisitions of the fingers
        @return tuple of left finger joint position and right finger 
                joint position'''
        return (self.left_finger.get_joint_position(), self.right_finger.get_joint_position())

    def GetEETransform(self):
        '''Get the end effector transform
        @return 4x4 transform of end effector in the world'''
        eeFrame = self.__robot.link_from_name('panda_hand')
        return pb_robot.geometry.tform_from_pose(eeFrame.get_link_pose())

class WSG50Hand_Real(object):
    #TODO this should be inside the abb node and should cover add srvs and topics. 
    # Do that as clean up later. 
    def __init__(self):
        print("Using WSG Hand")
        #TODO need to adjust srv.robot_GetRobotAngle
        # for open, call grasp. for close, call move (hacked for now)

    def Home(self):
        homeHand =  rospy.ServiceProxy('/wsg_50_driver/homing', srv.robot_GetRobotAngle)
        rospy.wait_for_service('/wsg_50_driver/homing', timeout=0.5)
        res = homeHand()

    def Move(self, width, speed=100):
        moveHand =  rospy.ServiceProxy('/wsg_50_driver/move', srv.robot_GetRobotAngle)
        rospy.wait_for_service('/wsg_50_driver/move', timeout=0.5)
        res = moveHand(width, speed)

    def Grasp(self, width, speed=100, force=40):
        setForceHand =  rospy.ServiceProxy('/wsg_50_driver/set_force', srv.robot_GetRobotAngle)
        rospy.wait_for_service('/wsg_50_driver/set_force', timeout=0.5)
        res = setForceHand(force)

        graspHand =  rospy.ServiceProxy('/wsg_50_driver/grasp', srv.robot_GetRobotAngle)
        rospy.wait_for_service('/wsg_50_driver/grasp', timeout=0.5)
        res = graspHand(width, speed)
