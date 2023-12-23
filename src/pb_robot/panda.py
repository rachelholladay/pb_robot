import random
import time
import numpy
import pybullet as p
import pb_robot
from .panda_controls import PandaControls

from pb_robot.ikfast.ikfast import closest_inverse_kinematics, ikfast_inverse_kinematics

class Panda(pb_robot.body.Body):
    '''Create all the functions for controlling the Panda Robot arm'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/franka_description/robots/panda_arm_hand.urdf'
        #self.urdf_file = 'models/franka_description/robots/panda_arm.urdf'

        with pb_robot.helper.HideOutput(): 
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=True)
        pb_robot.body.Body.__init__(self, self.id)

        self.arm_joint_names = ['panda_joint{}'.format(i) for i in range(1, 8)]
        self.arm_joints = [self.joint_from_name(n) for n in self.arm_joint_names]
        self.ik_info = pb_robot.ikfast.utils.IKFastInfo(module_name='franka_panda.ikfast_panda_arm',
                                                        base_link='panda_link0',
                                                        ee_link='panda_link8',
                                                        free_joints=['panda_joint7'])
        self.torque_limits = [87, 87, 87, 87, 12, 12, 12]
        self.startq = [0, -numpy.pi/4.0, 0, -0.75*numpy.pi, 0, numpy.pi/2.0, numpy.pi/4.0]
        self.hand = PandaHand(self.id)
        self.arm = Manipulator(self.id, self.arm_joints, self.hand, 'panda_hand', self.ik_info, self.torque_limits, self.startq)


class Manipulator(object):
    '''Class for Arm specific functions. Most of this is simply syntatic sugar for function
    calls to body functions. Within the documentation, N is the number of degrees of 
    freedom, which is 7 for Panda '''
    def __init__(self, bodyID, joints, hand, eeName, ik, torque_limits, startq=None):
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
        self.torque_limits = torque_limits

        # Eventually add a more fleshed out planning suite
        self.birrt = pb_robot.planners.BiRRTPlanner()
        self.snap = pb_robot.planners.SnapPlanner()
        self.cbirrt = pb_robot.planners.CBiRRTPlanner()

        # Add force torque sensor at wrist
        self.ft_joint = self.__robot.joint_from_name('panda_hand_joint')
        p.enableJointForceTorqueSensor(self.__robot.id, self.ft_joint.jointID, enableSensor=1)
        #self.control = PandaControls(self)

        # We manually maintain the kinematic tree of grasped objects by
        # keeping track of a dictionary of the objects and their relations
        # to the arm (normally the grasp matrix)
        self.grabbedRelations = dict()
        self.grabbedObjects = dict()

        # Use IK fast for inverse kinematics
        self.ik_info = ik

        # Set the robot to the default home position 
        if startq is not None:
            self.startq = startq #[0, -numpy.pi/4.0, 0, -0.75*numpy.pi, 0, numpy.pi/2.0, numpy.pi/4.0]
            self.SetJointValues(self.startq)
        self.collisionfn_cache = {}

        self.faces = []
        self.ee_offset = 0.12
        self.finger_offset = 0.08

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

        #If exists grabbed object, update its position too
        if len(self.grabbedObjects.keys()) > 0:
            hand_worldF = self.GetEETransform()
            for i in self.grabbedObjects.keys():
                obj = self.grabbedObjects[i]
                grasp_objF = self.grabbedRelations[i]
                obj_worldF = numpy.dot(hand_worldF, numpy.linalg.inv(grasp_objF))
                obj.set_transform(obj_worldF)

    def GetJointLimits(self):
        '''Return the upper and lower joint position limits
        @return 2xN Tuple of lower and upper joint limits'''
        return (self.__robot.get_min_limits(self.joints), 
                self.__robot.get_max_limits(self.joints))

    def Grab(self, obj, relation):
        '''Attach an object to the robot by storing the object and 
        its relative location to the robot arm. Now if we set
        the arm position, the object will move accordingly
        @param obj The object to be grabbed
        @param relation Transform of object relative to robot'''
        self.grabbedRelations[obj.get_name()] = relation
        self.grabbedObjects[obj.get_name()] = obj

    def Release(self, obj):
        '''Dettach an object by removing it from the grabbed object lists
        @param obj The object to be released'''
        self.grabbedObjects.pop(obj.get_name(), None)
        self.grabbedRelations.pop(obj.get_name(), None)

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

    def ComputeIK(self, transform, seed_q=None, force_seed=False, max_distance=0.2):
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
                                               self.eeFrame, pose, max_time=0.05), None)
        else:
            # Seeded IK uses the current ik value, so set that and then reset change
            old_q = self.GetJointValues()
            self.SetJointValues(seed_q)
            q = next(closest_inverse_kinematics(self.__robot, self.ik_info, self.eeFrame,
                                                pose, max_distance=max_distance, max_time=0.05), None)
            self.SetJointValues(old_q)
            # If no ik, fall back on unseed version
            if q is None and not force_seed:
                return self.ComputeIK(transform)
        return q 

    def get_collisionfn(self, obstacles=None, self_collisions=True):
        if obstacles is None:
            # If no set of obstacles given, assume all obstacles in the environment (that aren't the robot and not grasped)
            obstacles = [b for b in pb_robot.utils.get_bodies() if self.get_name() not in b.get_name()
                         and b.get_name() not in self.grabbedObjects.keys()]
        attachments = [g for g in self.grabbedObjects.values()]
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


    def GetJacobian(self, q): 
        '''Compute the jacobian at configuration q. The full set of joints
        (and hence jacobian calculation) includes the finger joints, so
        we remove those after
        @param q Configuration
        @return Nx6 array of J(q) '''
        allq = numpy.append(q, [0, 0]).tolist()
        (translate, rotate) = pb_robot.planning.compute_jacobian(self.__robot, self.eeFrame, positions=allq)
        jacobian = numpy.hstack((numpy.array(translate), numpy.array(rotate)))
        return numpy.transpose(jacobian[0:7, :])

    def GetCoriolosMatrix(self, q, dq):
        '''Compute C(q, q dot) by calling the inverse dynamics function with 
        zero acceleration. The full set of joints (and hence inverse dynamics 
        equations) include the finger joints, so we include them and then
        remove them after. Also, the inverse dynamics function cannot take 
        numpy array and must be fed lists.
        @param q Configuration
        @param dq joint velocities
        @return Coriolis vector, Nx1'''
        q_plus = numpy.append(q, [0, 0]).tolist()
        dq_plus = numpy.append(dq, [0, 0]).tolist()
        ddq = [0.0]*9
        coriolis = p.calculateInverseDynamics(self.__robot.id, q_plus, dq_plus, ddq)[0:7]
        return coriolis

    def InsideTorqueLimits(self, q, forces):
        '''Check if configuration is within torque limits, given force
        @param q Configuration
        @param force 6D array of force to check against
        @return check True if within torque limits '''
        jacobian = self.GetJacobian(q)
        torques = numpy.dot(numpy.transpose(jacobian), forces)
        inside = all(numpy.less(abs(torques), self.torque_limits)) # Assuming symmetric
        return inside

    def GetJointTorques(self):
        '''Read the joint torques simulated in pybullet
        @return List of the joint torques'''
        return [j.get_joint_torque() for j in self.joints]

    def GetJointVelocities(self):
        '''Read the joint velocities
        @return List of the joint velocities'''
        return numpy.array([j.get_joint_velocity() for j in self.joints])

    def GetFTWristReading(self):
        '''Read the 6D force torque simulated sensor at the wrist
        @return 6D tuple of forces and torques'''
        return p.getJointState(self.__robot.id, self.ft_joint.jointID)[2]

    def ExecutePositionPath(self, path, timestep=0.05):
        '''Simulate a configuration space path by incrementally setting the 
        joint values. This is instead of using control based methods
        #TODO add checks to insure path is valid. 
        @param path MxN list of configurations where M is number of positions
        @param timestep Wait time between each configuration ''' 
        for i in range(len(path)):
            self.SetJointValues(path[i])
            time.sleep(timestep)

    def getFaceByName(self, name):
        '''Search over the set of faces by name'''
        for f in self.faces:
            if name in f.name:
                return f
        return None

                        
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
