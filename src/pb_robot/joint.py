from collections import namedtuple
import pybullet as p
import pb_robot
import pb_robot.helper as helper
import pb_robot.geometry as geometry

CLIENT = 0

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags', 'jointDamping',
                                     'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])
JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
                                       'jointReactionForces', 'appliedJointMotorTorque'])

class Joint(object): # inherit what?
    def __init__(self, body, jointID):
        self.body = body
        self.bodyID = self.body.id
        self.jointID = jointID
        self.JointInfo = JointInfo
        self.JointState = JointState

    def __repr__(self):
        return self.get_joint_name()

    def get_joint_info(self):
        return self.JointInfo(*p.getJointInfo(self.bodyID, self.jointID, physicsClientId=CLIENT))

    def get_joint_name(self):
        return self.get_joint_info().jointName.decode('UTF-8')

    def get_joint_state(self):
        return self.JointState(*p.getJointState(self.bodyID, self.jointID, physicsClientId=CLIENT))

    def get_joint_position(self): 
        return self.get_joint_state().jointPosition

    def get_joint_velocity(self):
        return self.get_joint_state().jointVelocity

    def get_joint_reaction_force(self):
        return self.get_joint_state().jointReactionForces

    def get_joint_torque(self):
        return self.get_joint_state().appliedJointMotorTorque

    def get_joint_type(self):
        return self.get_joint_info().jointType

    def is_fixed(self):
        return self.get_joint_type() == p.JOINT_FIXED

    def is_movable(self):
        return not self.is_fixed()

    def is_circular(self):
        joint_info = self.get_joint_info()
        if joint_info.jointType == p.JOINT_FIXED:
            return False
        return joint_info.jointUpperLimit < joint_info.jointLowerLimit

    def get_joint_limits(self):
        if self.is_circular():
            # TODO: return UNBOUNDED_LIMITS
            return pb_robot.utils.CIRCULAR_LIMITS
        joint_info = self.get_joint_info()
        return joint_info.jointLowerLimit, joint_info.jointUpperLimit

    def get_min_limit(self):
        # TODO: rename to min_position
        return self.get_joint_limits()[0]

    def get_max_limit(self):
        return self.get_joint_limits()[1]

    def get_max_velocity(self):
        return self.get_joint_info().jointMaxVelocity

    def get_max_force(self):
        return self.get_joint_info().jointMaxForce

    def get_joint_q_index(self):
        return self.get_joint_info().qIndex

    def get_joint_v_index(self):
        return self.get_joint_info().uIndex

    def get_joint_axis(self):
        return self.get_joint_info().jointAxis

    def get_joint_parent_frame(self):
        joint_info = self.get_joint_info()
        return joint_info.parentFramePos, joint_info.parentFrameOrn

    def set_joint_position(self, value):
        p.resetJointState(self.bodyID, self.jointID, value, targetVelocity=0, physicsClientId=pb_robot.utils.CLIENT)

    def violates_limit(self, value):
        if self.is_circular():
            return False
        lower, upper = self.get_joint_limits()
        return (value < lower) or (upper < value)

    def wrap_position(self, position):
        if self.is_circular():
            return helper.wrap_angle(position)
        return position

    def get_joint_inertial_pose(self):
        dynamics_info = self.body.get_dynamics_info(self.jointID)
        return dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn
