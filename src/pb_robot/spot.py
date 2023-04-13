import random
import time
import numpy
import pybullet as p
import pb_robot
from transformations import translation_matrix, rotation_matrix, inverse_matrix, concatenate_matrices

class Spot(pb_robot.body.Body):
    '''Create all the functions for controlling the Panda Robot arm'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/spot_description/spot.urdf'

        with pb_robot.helper.HideOutput(): 
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=True)
        pb_robot.body.Body.__init__(self, self.id)

        self.eeName = 'arm_link_wr1'
        self.arm_joint_names = ['arm_sh0', 'arm_sh1', 'arm_el0', 'arm_el1', 'arm_wr0', 'arm_wr1']
        self.arm_joints = [self.joint_from_name(n) for n in self.arm_joint_names]

        self.hand = SpotHand(self.id, eeFrame=self.link_from_name(self.eeName))
        self.arm = Manipulator(self.id, self.arm_joints, self.hand, self.eeName)

class SpotArm(pb_robot.body.Body):
    '''Create all the functions for controlling the Panda Robot arm'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/spot_description/spot_arm.urdf'

        with pb_robot.helper.HideOutput():
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=True)
        pb_robot.body.Body.__init__(self, self.id)

        self.hand = SpotHand(self.id)
        self.arm_joint_names = ['arm_sh0', 'arm_sh1', 'arm_el0', 'arm_el1', 'arm_wr0', 'arm_wr1'] 
        self.arm_joints = [self.joint_from_name(n) for n in self.arm_joint_names]
        self.arm = Manipulator(self.id, self.arm_joints, self.hand, 'arm_link_wr1')

class Manipulator(object):
    '''Class for Arm specific functions. Most of this is simply syntatic sugar for function
    calls to body functions. Within the documentation, N is the number of degrees of 
    freedom, which is 7 for Panda '''
    def __init__(self, bodyID, joints, hand, eeName):
        '''Establish all the robot specific variables and set up key
        data structures. Eventually it might be nice to read the specific variables
        from a combination of the urdf and a yaml file'''
        self.bodyID = bodyID
        self.id = bodyID
        self.__robot = pb_robot.body.Body(self.bodyID)
        self.joints = joints
        self.jointsID = [j.jointID for j in self.joints]
        self.eeFrame = self.__robot.link_from_name(eeName)
        self.hand = hand

        # Use IK fast for inverse kinematics
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

    def ComputeFK(self, q, pose=None):
        '''Compute the forward kinematics of a configuration q
        @param configuration q
        @return 4x4 transform of the end effector when the robot is at
                configuration q'''
        if pose is None:
            pose = self.__robot.get_transform()

        old_q = self.GetJointValues()
        old_pose = self.__robot.get_transform()

        self.__robot.set_transform(pose)
        self.SetJointValues(q)
        pose = self.GetEETransform()

        self.SetJointValues(old_q)
        self.__robot.set_transform(old_pose)
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
            if self.IsCollisionFree(dofs, obstacles=[]): #self_collisions=True
                return dofs

    def ComputeIK(self, wrist_pose_worldF, robot_worldF=None, seed_q=None):
        '''Compute the analytic inverse kinematics of a transform, with the option
        to bias towards a seed configuration'''

        # shoulder to base (0, 0, 0) - that is height above floor
        SHOULDER_OFFSET = inverse_matrix(translation_matrix([0.292, 0., 0.873]))
        if robot_worldF is None:
            robot_worldF = self.get_transform()

        wrist_pose_robotF = numpy.dot(numpy.linalg.inv(robot_worldF), wrist_pose_worldF)
        wrist_pose_shoulderF = concatenate_matrices(SHOULDER_OFFSET, wrist_pose_robotF)
        min_limits, max_limits = self.GetJointLimits()

        solutions = analytic_spot_ik_6(wrist_pose_shoulderF, min_limits, max_limits)
        return select_solution(solutions, get_l1_distance, nearby_conf=seed_q)

    def get_collisionfn(self, obstacles=None, self_collisions=True):
        if obstacles is None:
            # If no set of obstacles given, assume all obstacles in the environment (that aren't the robot and not grasped)
            obstacles = [b for b in pb_robot.utils.get_bodies() if self.get_name() not in b.get_name()]
        attachments = []
        key = (frozenset(obstacles), frozenset(attachments), self_collisions)
        if key not in self.collisionfn_cache:
            self.collisionfn_cache[key] = pb_robot.collisions.get_collision_fn(
                self.__robot, self.joints, obstacles, attachments, self_collisions)
        return self.collisionfn_cache[key]

    def IsCollisionFree(self, q, obstacles=None, self_collisions=False):
        '''Check if a configuration is collision-free. Given any grasped objects
        we do not collision-check against those. 
        @param q Configuration to check at
        @param self_collisions Boolean on whether to include self-collision checks
        @return Boolean True if without collisions, false otherwise'''
        #FIXME Spot gives self-collision (seemingly incorrect) so we default turn it off

        # This is to cover that the collision function sets joints, but not using the arm version
        oldq = self.GetJointValues()
    
        collisionfn = self.get_collisionfn(obstacles=obstacles, self_collisions=self_collisions)

        # Evaluate if in collision
        val = not collisionfn(q)

        # Robot will error if links get too close (i.e. predicts collision)
        # so we want to insure that the is padding wrt collision-free-ness 
        distances = self.HasClearance(q)

        # Restore configuration
        self.SetJointValues(oldq)
        #print(val, distances)
        return val and distances

    def HasClearance(self, q, d=0.01):
        for i in self.__robot.all_links:
            for j in self.__robot.all_links:
                linkI = i.linkID
                linkJ = j.linkID
                # Dont want to check adjancent links. (and a few extra)
                if (abs(linkI-linkJ) < 2) or (linkI == 1 and linkJ == -1) or (linkJ == 0) or (linkI == 16 and linkJ == 14): 
                    break
                pts = p.getClosestPoints(self.__robot.id, self.__robot.id, distance=d, linkIndexA=linkI, linkIndexB=linkJ)
                if len(pts) > 0:
                    #print(i, linkI, j, linkJ)
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

                        
class SpotHand(pb_robot.body.Body):
    '''Set position commands for the Spot hand'''
    def __init__(self, bodyID=None, finger_name='arm_f1x', eeFrame=None):
        '''Pull fingers from robot's joint list'''

        '''
        # No separate hand URDF
        if bodyID is None:
            urdf_file = 'models/franka_description/robots/hand.urdf'
            with pb_robot.helper.HideOutput():
                with pb_robot.utils.LockRenderer():
                    bodyID = pb_robot.utils.load_model(urdf_file, fixed_base=True)
        '''

        self.bodyID = bodyID
        pb_robot.body.Body.__init__(self, bodyID)
        self.finger = self.joint_from_name(finger_name)
        self.joints = [self.finger]
        self.jointsID = [j.jointID for j in self.joints]
        if eeFrame is None:
            eeFrame = self.__robot.link_from_name('arm_link_wr1')
        self.eeFrame = eeFrame

    def Open(self):
        '''Open the fingers by setting their positions to the upper limit'''
        self.finger.set_joint_position(-1)

    def Close(self):
        '''Close the fingers by setting their positions to the inner limit'''
        self.finger.set_joint_position(0)

    def MoveTo(self, q):
        '''Move the hand to desired configuration q
        @param q Joint Configuration'''
        self.finger.set_joint_position(q)

    def GetJointPositions(self):
        '''Get the joint poisitions of the fingers
        @return tuple of left finger joint position and right finger 
                joint position'''
        return (self.finger.get_joint_position())

    def GetEETransform(self):
        '''Get the end effector transform
        @return 4x4 transform of end effector in the world'''
        return pb_robot.geometry.tform_from_pose(self.eeFrame.get_link_pose())

#########################################
# Spot IK Utils
#########################################
# This is a 3R elbow manipulator with a wrist with 3 intersecting axes
# That is, the simplest IK in the universe, solution below modeled on Hauser's boodk.

def analytic_spot_ik_6(wrist_pose_rel_shoulder, min_limits, max_limits):
    px, py, pz = wrist_pose_rel_shoulder[:3, 3]   # wrist position
    l2 = 0.3385
    l3 = numpy.sqrt(0.40330**2 + 0.0750**2)
    q3_off = numpy.arctan2(0.0750, 0.40330)
    # Solve the first three joints based on position
    q123_sols = []
    xl = numpy.sqrt(px**2 + py**2)
    q23 = IK2R(l2, l3, xl, -pz)
    if q23 is None:
        return []
    q1 = numpy.arctan2(py, px)
    for (q2, q3) in q23:
        q123_sols.append((q1, q2, q3 + q3_off))
    q23 = IK2R(l2, l3, -xl, -pz)
    assert q23 is not None
    q1 = q1 + numpy.pi
    for (q2, q3) in q23:
        q123_sols.append((q1, q2, q3 + q3_off))

    # Solve for the wrist angles
    qfull_sols = []
    for (q1, q2, q3) in q123_sols:
        r3_inv = inverse_matrix(
                  concatenate_matrices(rotation_matrix(q1, (0, 0, 1)),
                                       rotation_matrix(q2+q3, (0, 1, 0))))
        W = concatenate_matrices(r3_inv, wrist_pose_rel_shoulder)
        q5 = numpy.arccos(W[0, 0])
        for q in (q5, -q5):
            s5 = numpy.sin(q)
            q4 = numpy.arctan2(W[1, 0]/s5, -W[2, 0]/s5)
            q6 = numpy.arctan2(W[0, 1]/s5, W[0, 2]/s5)
            angles = (q1, q2, q3, q4, q, q6)
            if all_between(min_limits, angles, max_limits):
                qfull_sols.append(angles)
    return qfull_sols

def IK2R(L1, L2, x, y):
    xy_sq = x**2 + y**2
    c2 = (xy_sq - L1**2 - L2**2)/(2*L1*L2)
    if abs(c2) > 1:
        return None
    elif c2 == 1.0:
        return [(numpy.arctan2(y, x), 0)]
    elif c2 == -1.0 and xy_sq != 0.:
        return [(numpy.arctan2(y, x), numpy.pi)]
    elif c2 == -1.0 and xy_sq == 0.:
        return [(q1, numpy.pi) for q1 in [0, 2*numpy.pi]]
    else:
        q2_1 = numpy.arccos(c2)
        q2_2 = -q2_1
        theta = numpy.arctan2(y, x)
        q1q2 = [(theta - numpy.arctan2(L2*numpy.sin(q2_i), L1+L2*numpy.cos(q2_i)), q2_i) \
                for q2_i in (q2_1, q2_2)]
        for q1, q2 in q1q2:
            xk = (L1*numpy.cos(q1) + L2*numpy.cos(q1 + q2))
            yk = (L1*numpy.sin(q1) + L2*numpy.sin(q1 + q2))
            # print(f'xk={xk}, yk={yk}')
            assert abs(x - xk) < 0.0001
            assert abs(y - yk) < 0.0001
        return q1q2

def angle_diff(x, y):
    twoPi = 2*numpy.pi
    z = (x - y) % twoPi
    return z - twoPi if z > numpy.pi else z

def get_l1_distance(sol1, sol2):   # max angle difference
    maxDiff = 0.0
    diff = 0.0
    for i in range(len(sol1)):
        diff = abs(angle_diff(sol1[i], sol2[i]))
        if diff > maxDiff:
            maxDiff = diff
    return maxDiff

def all_between(lower_limits, values, upper_limits):
    assert len(lower_limits) == len(values)
    assert len(values) == len(upper_limits)
    return numpy.less_equal(lower_limits, values).all() and \
           numpy.less_equal(values, upper_limits).all()

def select_solution(solutions, get_distance, nearby_conf_angles=None, **kwargs):
    if not solutions:
        return None
    if nearby_conf_angles is None:
        return random.choice(solutions)
    return min(solutions, key=lambda conf: get_distance(nearby_conf_angles, conf, **kwargs))
