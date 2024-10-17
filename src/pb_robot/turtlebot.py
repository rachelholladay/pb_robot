import random
import time
import numpy
import pb_robot

class Turtlebot(pb_robot.body.Body):
    '''Create all the functions for controlling the Panda Robot arm'''
    def __init__(self):
        '''Generate the body and establish the other classes'''
        self.urdf_file = 'models/turtlebot/turtlebot_holonomic.urdf'
        #self.urdf_file = 'models/turtlebot/roomba.urdf'

        with pb_robot.helper.HideOutput(): 
            with pb_robot.utils.LockRenderer():
                self.id = pb_robot.utils.load_model(self.urdf_file, fixed_base=False)
        pb_robot.body.Body.__init__(self, self.id)

        # Set up Joints
        self.all_joints = self.joints
        self.movable_joints = [j for j in self.joints if not j.is_fixed()]
        self.joint_names = ['x', 'y', 'theta']
        self.joints = [self.joint_from_name(n) for n in self.joint_names]
        self.startq = [0, 0, 0]

        self.baselink = self.links[2]

        # Eventually add a more fleshed out planning suite
        self.birrt = pb_robot.planners.BiRRTPlanner()
        self.snap = pb_robot.planners.SnapPlanner()
        self.cbirrt = pb_robot.planners.CBiRRTPlanner()

        # We manually maintain the kinematic tree of grasped objects by
        # keeping track of a dictionary of the objects and their relations
        # to the arm (normally the grasp matrix)
        self.grabbedRelations = dict()
        self.grabbedObjects = dict()

        self.collisionfn_cache = {}

    def get_configuration(self):
        '''Return the robot configuration
        @return Nx1 array of joint values'''
        return self.GetJointValues()

    def set_configuration(self, q):
        return self.SetJointValues(q) 

    def get_transform(self):
        #return self.get_transform()
        print('Be careful of that!')
        return self.baselink.get_link_pose()

    #def set_transform(self, tform):
    #    return self.set_transform(tform)

    def GetJointValues(self):
        '''Return the robot configuration
        @return Nx1 array of joint values'''
        return numpy.array(self.get_joint_positions(self.joints))
    
    def SetJointValues(self, q):
        '''Set the robot to configuration q. Update the location of any
        grasped objects.
        @param Nx1 desired configuration'''
        self.set_joint_positions(self.joints, q)

        #If exists grabbed object, update its position too
        if len(self.grabbedObjects.keys()) > 0:
            hand_worldF = self.get_transform() #self.GetEETransform()
            for i in self.grabbedObjects.keys():
                obj = self.grabbedObjects[i]
                grasp_objF = self.grabbedRelations[i]
                obj_worldF = numpy.dot(hand_worldF, numpy.linalg.inv(grasp_objF))
                obj.set_transform(obj_worldF)

    def GetJointLimits(self):
        '''Return the upper and lower joint position limits
        @return 2xN Tuple of lower and upper joint limits'''
        return (self.get_min_limits(self.joints), 
                self.get_max_limits(self.joints))

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

    def get_collisionfn(self, obstacles=None, self_collisions=True):
        if obstacles is None:
            # If no set of obstacles given, assume all obstacles in the environment (that aren't the robot and not grasped)
            obstacles = [b for b in pb_robot.utils.get_bodies() if self.get_name() not in b.get_name()
                         and b.get_name() not in self.grabbedObjects.keys()]
        attachments = [g for g in self.grabbedObjects.values()]
        key = (frozenset(obstacles), frozenset(attachments), self_collisions)
        if key not in self.collisionfn_cache:
            self.collisionfn_cache[key] = pb_robot.collisions.get_collision_fn(
                self, self.joints, obstacles, attachments, self_collisions)
        return self.collisionfn_cache[key]

    def IsCollisionFree(self, q, obstacles=None, self_collisions=True):
        '''Check if a configuration is collision-free. Given any grasped objects
        we do not collision-check against those. 
        @param q Configuration to check at
        @param self_collisions Boolean on whether to include self-collision checks
        @return Boolean True if without collisions, false otherwise'''

        #TODO need to fix this, says always in collision
        collisionfn = self.get_collisionfn(obstacles=obstacles, self_collisions=self_collisions)
        # Evaluate if in collision
        val = not collisionfn(q)
        print(val)
        return val

    def ExecutePositionPath(self, path, timestep=0.05):
        '''Simulate a configuration space path by incrementally setting the 
        joint values. This is instead of using control based methods
        #TODO add checks to insure path is valid. 
        @param path MxN list of configurations where M is number of positions
        @param timestep Wait time between each configuration ''' 
        for p in path:
            self.SetJointValues(p)
            time.sleep(timestep)
