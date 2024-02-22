#/usr/bin/env python
# -*- coding: utf-8 -*-

'''CBiRRT between two random configurations'''

import random
import time
from scipy import spatial
import networkx as nx 
import numpy
import pb_robot_spot
import pybullet
from . import util
from .plannerTypes import GoalType, ConstraintType

class CBiRRTPlanner(object):
    '''My implementation of cbirrt, for now without constraining,
    for toying around'''
    def __init__(self):
        ## Constants 
        self.TOTAL_TIME = 20.0
        self.SHORTEN_TIME = 2.0 # For video level, 4 seconds
        self.PSAMPLE = 0.2 
        self.QSTEP = 1
        self.tstart = None

        self.goal = None
        self.goal_type = None
        self.constraints = []
        self.grasp = None
        self.manip = None
        self.goalTSRs = None
        self.pathTSRs = None

    def PlanToConfiguration(self, manip, start, goal_config, **kw_args):
        '''Plan from one joint location (start) to another (goal_config) with
        optional constraints. 
        @param manip Robot arm to plan with 
        @param start Joint pose to start from
        @param goal_config Joint pose to plan to
        @return OpenRAVE joint trajectory or None if plan failed'''
        path = self.CBiRRTPlanner(manip, start, goal_config, GoalType.JOINT, **kw_args)
        return util.generatePath(path)

    def PlanToEndEffectorPose(self, manip, start, goal_pose, **kw_args):
        '''Plan from one joint location (start) to an end effector pose (goal_pose) with
        optional constraints. The end effector pose is converted to a point TSR
        and passed off to a higher planner
        @param manip Robot arm to plan with 
        @param start Joint pose to start from
        @param goal_pose End effector pose to plan to
        @return OpenRAVE joint trajectory or None if plan failed'''
        tsr_chain = util.CreateTSRFromPose(manip, goal_pose)
        return self.PlanToEndEffectorTSR(manip, start, [tsr_chain], **kw_args)

    def PlanToEndEffectorPoses(self, manip, start, goal_poses, **kw_args):
        '''Plan from one joint location (start) to any pose within a set of 
        end effector pose (goal_poses) with optional constraints. The end effector 
        poses are converted into a chain of point TSRs and passed off to a
        higher planner
        @param manip Robot arm to plan with 
        @param start Joint pose to start from
        @param goal_poses Set of end effector pose to plan to
        @return OpenRAVE joint trajectory or None if plan failed'''
        chains = []
        for goal in goal_poses:
            tsr_chain = util.CreateTSRFromPose(manip, goal)
            chains.append(tsr_chain)
        return self.PlanToEndEffectorTSR(manip, start, chains, **kw_args)
    
    def PlanToEndEffectorTSR(self, manip, start, goal_tsr, constraints=[], **kw_args):
        '''Plan from one joint location (start) to any pose defined by a TSR
        on the end effector (goal_tsr) with optional constraints. There is also
        the option to "back in" to the goal position - instead of planning directly
        to the goal we plan to a pose slightly "backed up" from it and then execute
        the straight short motion from our backed up pose to our goal pose. This 
        has shown to make collision checking easier and work well when approaching
        objects. This process involves planning two paths, which are then stitched
        together. 
        @param manip Robot arm to plan with 
        @param start Joint pose to start from
        @param goal_poses Set of end effector pose to plan to
        @return OpenRAVE joint trajectory or None if plan failed'''
        path = self.CBiRRTPlanner(manip, start, goal_tsr, GoalType.TSR_EE, constraints=constraints, **kw_args)
        return util.generatePath(path)

    def CBiRRTPlanner(self, manip, start, goalLocation, goal_type, obstacles=None, constraints=[], grasp=None, pathTSRs=None):
        '''Given start and end goals, plan a path
        @param manip Arm to plan wit
        @param start start joint configuration
        @param goalLocation end goal, either configuration or TSR
        @param goal_type Configuration, TSR or Tool TSR
        @param constraints List of constraint functions given in form
                 [fn() ConstraintType]. List can have multiple of same type
        @param grasp The transform of the hand in the tool frame
        @param backupAmount Quantity, in meters to, to back up. If zero, we
                      do not back up in the goal position
        @param backupDIr Direction to back. Currently we only accept a single
                      direction ([1, 0, 0], [0, 1, 0], [0, 0, 1])
        @param path Given as series of waypoints to be converted to OpenRave trajectory'''

        # Turn off pybullet rendering for speed up XXX
        #pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, False)

        self.manip = manip
        self.goal = goalLocation
        self.goal_type = goal_type
        self.constraints = constraints
        self.grasp = grasp 
        self.obstacles = obstacles
        self.pathTSRs = pathTSRs
        original_pose = manip.GetJointValues()
        if self.goal_type == GoalType.TSR_TOOL and self.grasp is None:
            raise ValueError("Planning calls that operate on the tool require the grasp is given")

        if self.goal_type == GoalType.TSR_EE:
            self.goalTSRs = goalLocation

        # Create two trees
        Ta = nx.Graph(name='start')
        Tb = nx.Graph(name='goal') 
      
        Ta.add_node('0s', config=start)

        # If our goal is a joint configuration, that is our final goal.
        # However, if we have a TSR we create a dummy node
        if self.goal_type == GoalType.JOINT:
            Tb.add_node('0g', config=self.goal)
        else:
            dummy_values = numpy.ones((len(manip.GetJointValues())))*numpy.inf
            Tb.add_node('0g', config=dummy_values)
        path_array = self.plan(Ta, Tb)

        # Reset DOF Values and turn rendering back on XXX
        manip.SetJointValues(original_pose) 
        #pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, True)

        # Return a piecewise linear path
        return path_array

    def plan(self, Ta, Tb):
        '''Continue sampling and connecting till path or timeout'''
        self.tstart = time.time()

        while (time.time() - self.tstart) < self.TOTAL_TIME:
            (Tgoal, Tstart) = self.getGoalAndStartTree(Ta, Tb)
            if self.goal_type is not GoalType.JOINT and (Tgoal.number_of_nodes() == 1 or random.random() < self.PSAMPLE):
                Tgoal = self.addRootConfiguration(Tgoal)
                # Update trees - (more efficient way?)
                Ta = Tgoal.copy()
                Tb = Tstart.copy() 
            else:
                # RRT constrained extend
                q_rand = self.randomConfig()
                qa_near = self.nearestNeighbor(Ta, q_rand)
                #print("A Constrained Extend")
                (Ta, qa_reach) = self.constrainedExtend(Ta, qa_near, q_rand)
                qb_near = self.nearestNeighbor(Tb, Ta.nodes[qa_reach]['config'])
                #print("B Constrained Extend")
                (Tb, qb_reach) = self.constrainedExtend(Tb, qb_near, Ta.nodes[qa_reach]['config']) 
                if numpy.array_equal(Ta.nodes[qa_reach]['config'], Tb.nodes[qb_reach]['config']):
                    P = self.extractPath(Ta, qa_reach, Tb, qb_reach)
                    return self.shortenPath(P)
                else:
                    # Swap the two trees 
                    Ttemp = Ta.copy()
                    Ta = Tb.copy()
                    Tb = Ttemp.copy()
        # No path found
        return None 

    def getGoalAndStartTree(self, Ta, Tb):
        '''Return the tree with the name goal'''
        if Ta.graph['name'] == 'goal':
            return (Ta, Tb)
        elif Tb.graph['name'] == 'goal':
            return (Tb, Ta)
        else:
            return ValueError('Neither trees are goal trees!')

    def addRootConfiguration(self, T):
        '''Add goal configurations if the goal is not a single configuration. 
        We sample the end effector set and compute IK, only adding it if
        it satifies the relevant constraints
        @param T tree to add to
        @param T tree with new goal node'''
        if self.goal_type is GoalType.JOINT:
            raise TypeError("Cant add root configuration for non tsr-goal")

        #print("Adding root")
        targets = util.SampleTSRForPose(self.goalTSRs)

        # Get the IK config for pose (and its correspond c values for that chain
        #(qs, c) = GetInitialGuess()
        #(new_qs, new_c) = ConstrainConfig(None, qs, c, pose)
        qs = self.manip.ComputeIK(targets)
        qs = self.constrainConfig(None, qs, targets)
        if qs is not None:
            # Found goal configuration, add it to the tree
            name = self.getNextIdx(T)
            T.add_node(name, config=qs)
            T.add_edge(name, '0g')
        return T

    def constrainedExtend(self, T, q_near, q_target):
        '''Starting from q_near, aim towards q_target as far as you can
        @param T tree to be extending from
        @param q_near Node within T to start from 
        @param q_target Config of the target to grow towards'''
        print("Constrained Extend")
        qs = q_near
        qs_old = q_near
        while True:
            if numpy.array_equal(q_target, T.nodes[qs]['config']):
                return (T, qs) # Reached target
            elif numpy.linalg.norm(numpy.subtract(q_target, T.nodes[qs]['config'])) > numpy.linalg.norm(numpy.subtract(T.nodes[qs_old]['config'], q_target)):
                return (T, qs_old) # Moved further away

            qs_old = qs
            dist = numpy.linalg.norm(numpy.subtract(q_target, T.nodes[qs]['config']))
            qs_config = T.nodes[qs]['config'] + min(self.QSTEP, dist)*(numpy.subtract(q_target, T.nodes[qs]['config']) / dist)

            # Call projection enforcing check if relevant
            if self.pathTSRs is not None: 
                qs_config = self.constrainConfig(T.nodes[qs]['config'], qs_config)

            if qs_config is not None and self.checkEdgeCollision(T.nodes[qs_old]['config'], qs_config):
                qs = self.getNextIdx(T)
                T.add_node(qs, config=qs_config)
                T.add_edge(qs_old, qs)
                print('Regular Node {}'.format(qs_config))
            else:
                print("Projection failed")
                return (T, qs_old)

    def constrainConfig(self, qs_old, qs, targets=[]): 
        '''
        A "grasp" in this setup is a tsr chain (of length 1) thats a pose constraint
        on the entire path. (in this case, the virtual joint it makes has a Bw of all zeros, 
        i.e. for now we allow no motion through this joint). But this makes it easy in the future
        to assume a scenario where either have caged grasp (that does have movement) or any
        other type of grasp with a level of movement 

        c is the vector of TSR chain joint values of all TSR chains. 
        every configuration has a corresponding c vector and we store it
        because it is good initial guess for planning TSR chain joint values. 

        to clarify: q is the configuration of the robot manipulator
                    c is the configuration of the virual manipulator represented by TSR chains
        Should have c has a parameter for each node (in cases where have tsr)
        '''
        checkDist = False

        # if you call from AddRoot, the "target" is a transform sampled from your goal tsr
        if len(targets) == 0:
            checkDist = True
            # Should loop over the number of TSR chains the manipulator is dealing with
            # For now assume we only have one
            t0_s = self.manip.ComputeFK(qs) 
            targets = self.getClosestTransform(self.pathTSRs, t0_s)
            #print("computing target. distance from t0_s? {}".format(pb_robot.geometry.GeodesicDistance(t0_s, targets)))
            #input("next")

        qs = self.updatePhysicalConstraint(qs)
        qs_new = self.projectConstraint(qs, targets, pathTSR=checkDist)
        if qs_new is None or (checkDist and (numpy.linalg.norm(numpy.subtract(qs_new, qs_old)) > 2*self.QSTEP)):
            return None

        # Here is where we could rejection sample check for other constraints 
        pose_new = self.manip.ComputeFK(qs_new)
        if checkDist: 
            # Pathwise constraints, called from ConstrainedExtend
            ee_constraint = self.evaluateConstraints(ConstraintType.PATH_EE, pose=pose_new)
            joint_constraint = self.evaluateConstraints(ConstraintType.PATH_JOINT, config=qs_new, pose=pose_new)
        else:
            # Goalwise constraints. called from AddRoot
            ee_constraint = self.evaluateConstraints(ConstraintType.GOAL_EE, pose=pose_new)
            joint_constraint = self.evaluateConstraints(ConstraintType.GOAL_JOINT, config=qs_new, pose=pose_new)
        if not (ee_constraint and joint_constraint):
            return None

        return qs_new 

    def getClosestTransform(self, chain, t0_s):
        '''Get this closest transform to t0_s thats on chain.

         Normally this function takes in a c and returns a new c. 
        #If wanted to pass around c, the c returned here would be Bw_sample'''

        # Use TSR library to get the closest Bw value (as xyzrpy) to transform t0_s
        dist, Bw_sample = chain.distance(t0_s) 

        # Convert Bw value to a transform
        t0_sample = chain.to_transform(Bw_sample)

        return t0_sample

    def updatePhysicalConstraint(self, qs):
        '''Handles the constraints that come from physical joints
        Right now, not doing anything with this'''
        return qs

    def projectConstraint(self, q, targets, pathTSR):
        if pathTSR:
            tsrs = self.pathTSRs
        else:
            tsrs = self.goalTSRs[0]

        epsilon = 1e-3
        count = 50 # instead of while true loop, cut this off
        for i in range(count):
            # Compute distance from TSRs
            T0_s = self.manip.ComputeFK(q)                
            delta_x, bw = tsrs.distance(T0_s)
            # T0_vee is closest point on TSR to T0_s
            T0_vee = tsrs.to_transform(bw)
            
            # Compute distance between T0_s and T0_vee
            Tvee_s = numpy.dot(numpy.linalg.inv(T0_vee), T0_s)
            dvee_s = [Tvee_s[0, 3], Tvee_s[1, 3], Tvee_s[2, 3], numpy.arctan2(Tvee_s[2, 1], Tvee_s[2, 2]), -numpy.arcsin(Tvee_s[2, 0]), numpy.arctan2(Tvee_s[1, 0], Tvee_s[0, 0])]
            delta_x = numpy.linalg.norm(dvee_s)

            if delta_x < epsilon: return q
            J = self.manip.GetJacobian(q)
            delta_q_error = numpy.dot(numpy.dot(numpy.transpose(J), numpy.linalg.inv(numpy.dot(J, numpy.transpose(J)))), dvee_s)
            
            q -= delta_q_error
            q = self.clampJointLimits(q)
            if pathTSR:
                pb_robot_spot.viz.draw_pose(pb_robot_spot.geometry.pose_from_tform(T0_s), length=0.2, width=2)
                pb_robot_spot.viz.draw_pose(pb_robot_spot.geometry.pose_from_tform(T0_vee), length=0.2, width=10)
                print(bw)
                print(dvee_s)
                #TODO geodesic error gives x, y, z, theta. Maybe, try using the x, y, z from this metric to see if it improves the projection step..?
                print(pb_robot_spot.geometry.GeodesicError(T0_s, T0_vee))
                print(delta_x, pb_robot_spot.geometry.GeodesicDistance(T0_s, T0_vee))
                input("Next iteration?")
            pb_robot_spot.viz.remove_all_debug() 
        return None
         
    def evaluateConstraints(self, constraintType, **kw_args):
        '''Given the list of constraints and the constraint time, 
        find all constraints of that type and evaluate them
        @param constraintType type to check
        @param kw_args the parameters of the constraint function
        @return True if constraints are satified'''
        # Grab all the relevant constraints
        relevantConstraints = [self.constraints[i][0] for i in range(len(self.constraints)) if self.constraints[i][1] == constraintType]
        
        # If there are no relevant constraints, also satified
        if len(relevantConstraints) == 0:
            return True

        # Evaluate constraints
        return all((fn(**kw_args) for fn in relevantConstraints))

    def clampJointLimits(self, qs):
        '''Given a proposed next joint space location, check that it is within
        joint limits. If it is not, clamp it to the nearest joint limit
        (joint wise)
        @param qs Joint configuration
        @param qs_new Joint configuration'''
        (lower, upper) = self.manip.GetJointLimits()
        qs_new = [max(lower[i], min(qs[i], upper[i])) for i in range(len(lower))]
        return qs_new

    def checkEdgeCollision(self, q_parent, q):
        '''Check if path from q_first to q_second is collision free
        @param q Joint configuration
        @return collisionFree (boolean) True if collision free''' 
        # Reject if end point is not collision free
        if not self.manip.IsCollisionFree(q, obstacles=self.obstacles):
            return False
        cdist = util.cspaceLength([q_parent, q])
        count = int(cdist / 0.1) # Check every 0.1 distance (a little arbitrary)

        # linearly interpolate between that at some step size and check all those points
        interp = [numpy.linspace(q_parent[i], q[i], count+1).tolist() for i in range(len(q))]
        middle_qs = numpy.transpose(interp)[1:-1] # Remove given points
        return all((self.manip.IsCollisionFree(m, obstacles=self.obstacles) for m in middle_qs)) 

    def randomConfig(self):
        '''Sample a random configuration within the reachable c-space.
        Random values between joint values
        @return Random configuration within joint limits'''
        (lower, upper) = self.manip.GetJointLimits()
        joint_values = numpy.zeros(len(lower))
        for i in range(len(lower)):
            joint_values[i] = random.uniform(lower[i], upper[i])
        return joint_values

    def nearestNeighbor(self, T, q_rand):
        '''Find nearest neighbor of q_rand in T using euclidean distance in
        joint space'''
        nodes = list(T.nodes())
        tree_dists = [T.nodes[n]['config'] for n in nodes]
        dists = spatial.distance.cdist(tree_dists, [q_rand], metric='euclidean')
        closest_q = nodes[numpy.argmin(dists)]
        return closest_q

    def extractPath(self, Ta, qa_reach, Tb, qb_reach):
        '''We have paths from 0 to each reach where the reaches are equal,
        therefore we connect the two trees to make one path
        @param Ta first tree
        @param qa_reach end point on first tree (node name)
        @param Tb second tree
        @param qb_reach end point on second tree (node name)
        @return path Array of joint values representing the path'''
        a_pts = nx.shortest_path(Ta, source='0'+Ta.graph['name'][0], target=qa_reach)
        a_distance = [Ta.nodes[x]['config'] for x in a_pts]
        b_pts = nx.shortest_path(Tb, source='0'+Tb.graph['name'][0], target=qb_reach)
        b_distance = [Tb.nodes[x]['config'] for x in b_pts] 

        if Ta.graph['name'] == 'start':
            path = a_distance + b_distance[::-1]
        else:
            path = b_distance + a_distance[::-1]

        # If using goal tsr, remove dummy node
        if self.goal_type is not GoalType.JOINT: 
            path = path[0:len(path)-1]
        return numpy.array(path)

    def shortenPath(self, P):
        '''Given time remaining, we randomly sample to see if we can shorten
        the path. For a random subsection of the path, we see if we can
        replace it with a shorter path
        @param P current path represnted as array of joint poses
        @param P new path (same representation) that is equal lenght or less'''
        shortenStart = time.time()
        #while ((time.time() - self.tstart) < self.TOTAL_TIME) and ((time.time() - shortenStart) < shortenTime):
        while (time.time() - shortenStart) < self.SHORTEN_TIME:
            if len(P) < 3: return P # Too few waypoints to shortcut
            Tshortcut = nx.DiGraph(name='postprocessing')
            i = random.randint(0, len(P)-2)
            j = random.randint(i, len(P)-1)
            Tshortcut.add_node('0p', config=P[i])
            (newT, qreach) = self.constrainedExtend(Tshortcut, '0p', P[j])
            if numpy.array_equal(newT.nodes[qreach]['config'], P[j]):
                old_length = util.cspaceLength(P[i:j+1])
                newNodePath = nx.shortest_path(newT, '0p', qreach)
                newJointPath = [newT.nodes[x]['config'] for x in newNodePath]
                new_length = util.cspaceLength(newJointPath)
                if new_length < old_length:
                    P = numpy.vstack((P[0:i], newJointPath, P[j+1:]))
        return P

    def getNextIdx(self, T):
        '''Given tree, search node values and give the next node value'''
        return str(max([int(x[:-1]) for x in T.nodes()])+1) + T.graph['name'][0] 
