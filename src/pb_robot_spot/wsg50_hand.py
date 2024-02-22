import numpy
import pybullet as p
import pb_robot_spot

class WSG50Hand(pb_robot_spot.body.Body):
    '''Set position commands for the panda hand. Have not yet included
    gripping with force.'''
    def __init__(self, bodyID=None, left_finger_name='base_joint_gripper_left', right_finger_name='base_joint_gripper_right'):
        '''Pull left and right fingers from robot's joint list'''
        if bodyID is None:
            urdf_file = 'models/wsg50_description/wsg_50.urdf'
            with pb_robot_spot.helper.HideOutput():
                with pb_robot_spot.utils.LockRenderer():
                    bodyID = pb_robot_spot.utils.load_model(urdf_file, fixed_base=True)

        pb_robot_spot.body.Body.__init__(self, bodyID)
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
        return pb_robot_spot.geometry.tform_from_pose(eeFrame.get_link_pose())

#TODO want to move to wsg50_common and proper imports (not command line calls)
import os
#from wsg_50_common import msg

class WSG50HandReal(object):
    def __init__(self):
        import rospy
        # If rosnode is not running, start one
        if 'unnamed' in rospy.get_name():
            rospy.init_node('wsg50_node', anonymous=True)

        self.openValue = 110
        self.closeValue = 0

    def home(self):
        os.system("rosservice call /wsg_50_driver/homing")

    def move(self, width, speed=50):
        os.system("rosservice call /wsg_50_driver/move {} {}".format(width, speed))

    def open(self, speed=50):
        os.system("rosservice call /wsg_50_driver/move {} {}".format(self.openValue, speed))

    def close(self, speed=50):
        os.system("rosservice call /wsg_50_driver/move {} {}".format(self.closeValue, speed))

    def grasp(self, width, force, speed=50):
        os.system("rosservice call /wsg_50_driver/set_force {}".format(force))
        os.system("rosservice call /wsg_50_driver/move {} {}".format(width, speed))

    def get_width(self):
        import rospy
        try:
            hand_status = rospy.wait_for_message("wsg_50_driver/status", msg.Status, timeout=2)
            return hand_status.width
        except (rospy.ROSException, rospy.ROSInterruptException):
            print("Unable to contact Hand")
            return 0 
