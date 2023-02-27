import random
import time
import numpy
import pybullet as p
import pb_robot
from .panda_controls import PandaControls

from pb_robot.ikfast.ikfast import closest_inverse_kinematics, ikfast_inverse_kinematics

class Spot(pb_robot.body.Body):
    '''Create all the functions for controlling the Panda Robot arm'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/spot_description/spot.urdf'

        with pb_robot.helper.HideOutput(): 
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=True)
        pb_robot.body.Body.__init__(self, self.id)


class SpotArm(pb_robot.body.Body):
    '''Create all the functions for controlling the Panda Robot arm'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/spot_description/spot_arm.urdf'

        with pb_robot.helper.HideOutput():
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=True)
        pb_robot.body.Body.__init__(self, self.id)


        self.arm_joint_names = ['arm0.sh0', 'arm0.sh1', 'arm0.hr0', 'arm0.el0', 'arm0.el1', 'arm0.wr0', 'arm0.wr1'] 
        self.arm_joints = [self.joint_from_name(n) for n in self.arm_joint_names]
        self.ik_info = pb_robot.ikfast.utils.IKFastInfo(module_name='spot_arm.ikfast_spot_arm',
                                                        base_link='base',
                                                        ee_link='arm0.link_wr1',
                                                        free_joints=['arm0.wr1'])

        self.arm = Manipulator(self.id, self.arm_joints, 'arm0.link_wr1', self.ik_info)

        '''
        self.hand = PandaHand(self.id)
        self.arm = Manipulator(self.id, self.arm_joints, self.hand, 'panda_hand', self.ik_info, self.torque_limits, self.startq)
        '''

class Manipulator(object):
    '''Class for Arm specific functions. Most of this is simply syntatic sugar for function
    calls to body functions. Within the documentation, N is the number of degrees of 
    freedom, which is 7 for Panda '''
    def __init__(self, bodyID, joints, eeName, ik):
        '''Establish all the robot specific variables and set up key
        data structures. Eventually it might be nice to read the specific variables
        from a combination of the urdf and a yaml file'''
        self.bodyID = bodyID
        self.id = bodyID
        self.__robot = pb_robot.body.Body(self.bodyID)
        self.joints = joints
        self.jointsID = [j.jointID for j in self.joints]
        self.eeFrame = self.__robot.link_from_name(eeName)

        # Use IK fast for inverse kinematics
        self.ik_info = ik
        self.collisionfn_cache = {}
        self.startq = [0]*len(joints)

    def get_name(self):
        return self.__robot.get_name()

    def __repr__(self):
        return self.get_name() + '_arm'

    def get_configuration(self):
        '''Return the robot configuration
        @return Nx1 array of joint values'''
        return self.GetJointValues()

    def set_configuration(self, q):
        return self.SetJointValues(q) 

    def get_transform(self):
        return self.__robot.get_transform()

    def set_transform(self, tform):
        return self.__robot.set_transform(tform)

    def GetJointValues(self):
        '''Return the robot configuration
        @return Nx1 array of joint values'''
        return numpy.array(self.__robot.get_joint_positions(self.joints))
    
    def SetJointValues(self, q):
        '''Set the robot to configuration q. Update the location of any
        grasped objects.
        @param Nx1 desired configuration'''
        self.__robot.set_joint_positions(self.joints, q)

    def GetJointLimits(self):
        '''Return the upper and lower joint position limits
        @return 2xN Tuple of lower and upper joint limits'''
        return (self.__robot.get_min_limits(self.joints), 
                self.__robot.get_max_limits(self.joints))

    def GetEETransform(self):
        '''Get the end effector transform
        @return 4x4 transform of end effector in the world'''
        return pb_robot.geometry.tform_from_pose(self.eeFrame.get_link_pose())

    def ComputeFK(self, q, tform=None):
        '''Compute the forward kinematics of a configuration q
        @param configuration q
        @return 4x4 transform of the end effector when the robot is at
                configuration q'''
        if tform is not None:
            old_pose = self.get_transform()
            self.set_transform(tform)
        old_q = self.GetJointValues()
        self.SetJointValues(q)
        pose = self.GetEETransform()
        self.SetJointValues(old_q)
        if tform is not None:
            self.set_transform(tform)    
        return pose 

    def randomConfiguration(self):
        '''Generate a random configuration inside the position limits
        that doesn't have self-collision
        @return Nx1 configuration'''
        (lower, upper) = self.GetJointLimits()
        while True:
            dofs = numpy.zeros(len(lower))
            for i in range(len(lower)):
                dofs[i] = random.uniform(lower[i], upper[i])
            if self.IsCollisionFree(dofs, self_collisions=True, obstacles=[]):
                return dofs

    def ComputeIK(self, transform, seed_q=None, max_distance=0.2):
        '''Compute the inverse kinematics of a transform, with the option 
        to bias towards a seed configuration. If no IK can be found with that
        bias we attempt to find an IK without that bias
        @param transform 4x4 desired pose of end effector
        @param (optional) seed_q Configuration to bias the IK
        @return Nx1 configuration if successful, else 'None' '''

        #These function operate in transforms but the IK function operates in poses
        pose = pb_robot.geometry.pose_from_tform(transform)

        if seed_q is None:
            q = next(ikfast_inverse_kinematics(self.__robot, self.ik_info, 
                                               self.eeFrame, pose, max_time=1), None)
        else:
            # Seeded IK uses the current ik value, so set that and then reset change
            old_q = self.GetJointValues()
            self.SetJointValues(seed_q)
            q = next(closest_inverse_kinematics(self.__robot, self.ik_info, self.eeFrame,
                                                pose, max_distance=max_distance, max_time=0.05), None)
            self.SetJointValues(old_q)
            # If no ik, fall back on unseed version
            if q is None:
                return self.ComputeIK(transform)
        return q 

    def get_collisionfn(self, obstacles=None, self_collisions=True):
        if obstacles is None:
            # If no set of obstacles given, assume all obstacles in the environment (that aren't the robot and not grasped)
            obstacles = [b for b in pb_robot.utils.get_bodies() if self.get_name() not in b.get_name()
                         and b.get_name() not in self.grabbedObjects.keys()]
        attachments = []
        key = (frozenset(obstacles), frozenset(attachments), self_collisions)
        if key not in self.collisionfn_cache:
            self.collisionfn_cache[key] = pb_robot.collisions.get_collision_fn(
                self.__robot, self.joints, obstacles, attachments, self_collisions)
        return self.collisionfn_cache[key]

    def IsCollisionFree(self, q, obstacles=None, self_collisions=True, handJoint=None):
        '''Check if a configuration is collision-free. Given any grasped objects
        we do not collision-check against those. 
        @param q Configuration to check at
        @param self_collisions Boolean on whether to include self-collision checks
        @return Boolean True if without collisions, false otherwise'''
        # This is to cover that the collision function sets joints, but not using the arm version
        oldq = self.GetJointValues()
        self.SetJointValues(oldq)
        if handJoint is not None:
            (oldhand, _) = self.hand.GetJointPositions()
            self.hand.MoveTo(handJoint)

        collisionfn = self.get_collisionfn(obstacles=obstacles, self_collisions=self_collisions)

        # Evaluate if in collision
        val = not collisionfn(q)

        # Robot will error if links get too close (i.e. predicts collision)
        # so we want to insure that the is padding wrt collision-free-ness 
        distances = self.HasClearance(q)

        # Restore configuration
        self.SetJointValues(oldq)
        if handJoint is not None:
            self.hand.MoveTo(oldhand*2)
        return val and distances

    def HasClearance(self, q):
        for i in self.__robot.all_links:
            for j in self.__robot.all_links:
                linkI = i.linkID
                linkJ = j.linkID
                # Dont want to check adjancent links or link 8 (fake hand joint)
                if (abs(linkI-linkJ) < 2) or (linkI == 8) or (linkJ == 8):
                    break
                pts = p.getClosestPoints(self.__robot.id, self.__robot.id, distance=0.01, linkIndexA=linkI, linkIndexB=linkJ)
                if len(pts) > 0:
                    return False
        return True

    def GetJointTorques(self):
        '''Read the joint torques simulated in pybullet
        @return List of the joint torques'''
        return [j.get_joint_torque() for j in self.joints]

    def GetJointVelocities(self):
        '''Read the joint velocities
        @return List of the joint velocities'''
        return numpy.array([j.get_joint_velocity() for j in self.joints])

    def ExecutePositionPath(self, path, timestep=0.05):
        '''Simulate a configuration space path by incrementally setting the 
        joint values. This is instead of using control based methods
        #TODO add checks to insure path is valid. 
        @param path MxN list of configurations where M is number of positions
        @param timestep Wait time between each configuration ''' 
        for i in range(len(path)):
            self.SetJointValues(path[i])
            time.sleep(timestep)

                        
class PandaHand(pb_robot.body.Body):
    '''Set position commands for the panda hand. Have not yet included
    gripping with force.'''
    def __init__(self, bodyID=None, left_finger_name='panda_finger_joint1', right_finger_name='panda_finger_joint2'):
        '''Pull left and right fingers from robot's joint list'''
        if bodyID is None:
            urdf_file = 'models/franka_description/robots/hand.urdf'
            with pb_robot.helper.HideOutput():
                with pb_robot.utils.LockRenderer():
                    bodyID = pb_robot.utils.load_model(urdf_file, fixed_base=True)

        self.bodyID = bodyID
        pb_robot.body.Body.__init__(self, bodyID)
        self.left_finger = self.joint_from_name(left_finger_name)
        self.right_finger = self.joint_from_name(right_finger_name)
        self.joints = [self.left_finger, self.right_finger]
        self.jointsID = [j.jointID for j in self.joints]
        self.torque_limits = [50, 50] # Faked

    def Open(self):
        '''Open the fingers by setting their positions to the upper limit'''
        self.left_finger.set_joint_position(0.04)
        self.right_finger.set_joint_position(0.04)

    def Close(self):
        '''Close the fingers by setting their positions to the inner limit'''
        self.left_finger.set_joint_position(0)
        self.right_finger.set_joint_position(0)

    def MoveTo(self, distance):
        '''Move the fingers uniformally such that 'distance' is the width
        between the two fingers. Therefore, each each finger will move 
        distance/2. 
        @param distance Desired distance between fingers'''
        if not (0 <= distance <= 0.08):
            raise IOError("Invalid distance request. The value must be between 0 and 0.08")
        finger_distance = distance / 2.0
        self.left_finger.set_joint_position(finger_distance)
        self.right_finger.set_joint_position(finger_distance)

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
