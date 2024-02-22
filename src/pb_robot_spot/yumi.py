from collections import defaultdict, deque, namedtuple
import numpy as np
import pybullet as p
import pb_robot_spot
import geometry
import body

#TODO generalize such that this is robot class, store robot information separately

class Yumi(body.Body):
    def __init__(self, **kwargs):
        self.urdf_file = 'models/yumi_description/yumi.urdf'
        self.id = pb_robot_spot.utils.load_model(self.urdf_file, **kwargs) 
        body.Body.__init__(self, self.id)

        # TODO pull this information from like a yaml or setup file
        self.arm_names = ('left_arm', 'right_arm')
        self.left_arm_names = ['yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l']
        self.right_arm_names = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', 'yumi_joint_6_r']
        self.left_joints = [self.joint_from_name(n) for n in self.left_arm_names]
        self.right_joints = [self.joint_from_name(n) for n in self.right_arm_names]
        self.right_hand_name = 'gripper_r_base'
        self.left_hand_name = 'gripper_l_base'
        self.right_hand = self.link_from_name(self.right_hand_name)
        self.left_hand = self.link_from_name(self.left_hand_name)
        self.right_arm = YumiArm(self.id, self.right_joints, self.right_hand_name)
        self.left_arm = YumiArm(self.id, self.left_joints, self.left_hand_name)

class YumiArm(object):
    def __init__(self, bodyID, joints, handName):
        self.__robot = body.Body(bodyID)
        self.joints = joints #XXX not names, actual joints (change variable name)
        self.hand = self.__robot.link_from_name(handName)

    def GetJointValues(self):
        return self.__robot.get_joint_positions(self.joints)
    
    def SetJointValues(self, q):
        return self.__robot.set_joint_positions(self.joints, q)

    def GetJointLimits(self):
        return (self.__robot.get_min_limits(self.joints), 
                self.__robot.get_max_limits(self.joints))

    def GetEETransform(self):
        return geometry.tform_from_pose(self.hand.get_link_pose())

    def ComputeFK(self, q):
        old_q = self.GetJointValues()
        self.SetJointValues(q)
        pose = self.GetEETransform()
        self.SetJointValues(old_q)
        return pose 

    def ComputeIK(self, pose):
        #pose = geometry.pose_from_tform(transform)
        q = pb_robot_spot.utils.inverse_kinematics(self.__robot, self.hand, pose)
        return q 

    def RandomIK(self, transform, objName=None, relation=None):
        #TODO check about randomizing this
        #TODO implement objName, relation information 
        pose = geometry.pose_from_tform(transform)
        q = pb_robot_spot.utils.inverse_kinematics(self.__robot, self.hand, pose)
        #transform = geometry.tform_from_pose(q)
        #return transform
        return q

    def IsCollisionFree(self, q, objName=None, relation=None):
        return True #TODO need to write 
