#/usr/bin/env python
# -*- coding: utf-8 -*-

'''BiRRT between two random configurations'''

import random
import time
from scipy import spatial
import networkx as nx 
import numpy
from . import util
from .plannerTypes import GoalType, ConstraintType

class BiRRTPlanner(object):
    '''My implementation of cbirrt, for now without constraining,
    for toying around'''
    def __init__(self):
        ## Constants 
        self.TOTAL_TIME = 10.0
        self.SHORTEN_TIME = 2.0 # For video level, 4 seconds
        self.PSAMPLE = 0.2 
        self.QSTEP = 1
        self.tstart = None

        self.goal = None
        self.goal_type = None
        self.constraints = None
        self.grasp = None
        self.manip = None

        self.handles = []

    def PlanToConfiguration(self, manip, start, goal_config, **kw_args):
        '''Plan from one joint location (start) to another (goal_config) with
        optional constraints. 
        @param manip Robot arm to plan with 
        @param start Joint pose to start from
        @param goal_config Joint pose to plan to
        @return OpenRAVE joint trajectory or None if plan failed'''
        path = self.BiRRTPlanner(manip, start, goal_config, GoalType.JOINT, **kw_args)
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
        for i in range(len(goal_poses)):
            tsr_chain = util.CreateTSRFromPose(manip, goal_poses[i])
            chains.append(tsr_chain)
        return self.PlanToEndEffectorTSR(manip, start, chains, **kw_args)
    
    def PlanToEndEffectorTSR(self, manip, start, goal_tsr, constraints=None, **kw_args):
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
        path = self.BiRRTPlanner(manip, start, goal_tsr, GoalType.TSR_EE, constraints=constraints, **kw_args)
        return util.generatePath(path)

    def BiRRTPlanner(self, manip, start, goalLocation, goal_type, obstacles=None, constraints=None, grasp=None):
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
 
        self.manip = manip
        self.goal = goalLocation
        self.goal_type = goal_type
        self.constraints = constraints
        self.grasp = grasp 
        self.obstacles = obstacles
        original_pose = manip.GetJointValues()
        if self.goal_type == GoalType.TSR_TOOL and self.grasp is None:
            raise ValueError("Planning calls that operate on the tool require the grasp is given")

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
        (path_array, _, _) = self.plan(Ta, Tb)

        # Reset DOF Values
        manip.SetJointValues(original_pose) 

        # Return an openrave trajectory
        return path_array

    def plan(self, Ta, Tb):
        '''Continue sampling and connecting till path or timeout'''
        self.tstart = time.time()

        while (time.time() - self.tstart) < self.TOTAL_TIME:
            (Tgoal, Tstart) = self.getGoalAndStartTree(Ta, Tb)
            if self.goal_type is not GoalType.JOINT and (Tgoal.number_of_nodes() == 1 or random.random() < self.PSAMPLE):
                Tgoal = self.addRootConfiguration(Tgoal)
                # Update trees - (FIND MORE EFFICIENT METHOD)
                Ta = Tgoal.copy()
                Tb = Tstart.copy() 
            else:
                q_rand = self.randomConfig()
                qa_near = self.nearestNeighbor(Ta, q_rand)
                (Ta, qa_reach) = self.constrainedExtend(Ta, qa_near, q_rand)
                qb_near = self.nearestNeighbor(Tb, Ta.nodes[qa_reach]['config'])
                (Tb, qb_reach) = self.constrainedExtend(Tb, qb_near, Ta.nodes[qa_reach]['config']) 
                if numpy.array_equal(Ta.nodes[qa_reach]['config'], Tb.nodes[qb_reach]['config']):
                    P = self.extractPath(Ta, qa_reach, Tb, qb_reach)
                    return (self.shortenPath(P), Ta, Tb)
                else:
                    # Swap the two trees
                    Ttemp = Ta.copy()
                    Ta = Tb.copy()
                    Tb = Ttemp.copy()
        return (None, Ta, Tb) # No Path found

    def getGoalAndStartTree(self, Ta, Tb):
        '''Return the three with the name goal'''
        if Ta.graph['name'] == 'goal':
            return (Ta, Tb)
        elif Tb.graph['name'] == 'goal':
            return (Tb, Ta)
        else:
            return ValueError('Neither trees are goal trees!')

    def evaluateConstraints(self, constraintType, **kw_args):
        '''Given the list of constraints and the constraint time, 
        find all constraints of that type and evaluate them
        @param constraintType type to check
        @param kw_args the parameters of the constraint function
        @return True if constraints are satified'''
        # If there are no constraints, then they are trivially satified
        if self.constraints is None:
            return True

        # Grab all the relevant constraints
        relevantConstraints = [self.constraints[i][0] for i in range(len(self.constraints)) if self.constraints[i][1] == constraintType]
        
        # If there are no relevant constraints, also satified
        if len(relevantConstraints) == 0:
            return True

        # Evaluate constraints
        return all((fn(**kw_args) for fn in relevantConstraints))

    def addRootConfiguration(self, T):
        '''Add goal configurations if the goal is not a single configuration. 
        We sample the end effector set and compute IK, only adding it if
        it satifies the relevant constraints
        @param T tree to add to
        @param T tree with new goal node'''
        if self.goal_type is GoalType.JOINT:
            raise TypeError("Cant add root configuration for non tsr-goal")

        searching = True
        while searching and (time.time() - self.tstart) < self.TOTAL_TIME:
            # Sample a TSR and then sample an EE pose from that TSR
            pose = util.SampleTSRForPose(self.goal)
           
            # Transform by grasp if needed
            if self.goal_type is GoalType.TSR_TOOL:
                ee_pose = numpy.dot(pose, self.grasp)
            else:
                ee_pose = pose

            # If there is an ee constraint, check it
            if self.evaluateConstraints(ConstraintType.GOAL_EE, pose=ee_pose):
                config = self.manip.ComputeIK(ee_pose)
                if config is not None:
                    # if there is a joint constraint, check it
                    if self.evaluateConstraints(ConstraintType.GOAL_JOINT, config=config, pose=ee_pose):
                        searching = False

        # Timed out, no root to be added
        if searching:
            return T

        # Found goal configuration, add it to the tree
        name = self.getNextIdx(T)
        T.add_node(name, config=config)
        T.add_edge(name, '0g')
        return T

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

    def constrainedExtend(self, T, q_near, q_target):
        '''Starting from q_near, aim towards q_target as far as you can
        @param T tree to be extending from
        @param q_near Node within T to start from 
        @param q_target Config of the target to grow towards'''
        qs = q_near
        qs_old = q_near
        while True:
            if numpy.array_equal(q_target, T.nodes[qs]['config']):
                return (T, qs) # Reached target
            elif numpy.linalg.norm(numpy.subtract(q_target, T.nodes[qs]['config'])) > numpy.linalg.norm(numpy.subtract(T.nodes[qs_old]['config'], q_target)):
                return (T, qs_old) # Moved further away

            qs_old = qs
            dist = numpy.linalg.norm(numpy.subtract(q_target, T.nodes[qs]['config']))
            qs_config_proposed = T.nodes[qs]['config'] + min(self.QSTEP, dist)*(numpy.subtract(q_target, T.nodes[qs]['config']) / dist)
            qs_config = self.approveNewNode(qs_config_proposed, T.nodes[qs]['config'])
            if qs_config is not None:
                qs = self.getNextIdx(T)
                T.add_node(qs, config=qs_config)
                T.add_edge(qs_old, qs)
            else:
                return (T, qs_old)

    def approveNewNode(self, qs_proposed, qs_parent):
        '''We will only approve the new node on several conditions. We
        constrain it to be within joint limits. We reject if its in 
        collision or if it violents any path wide constraints
        @param qs_proposed joint position of proposed node
        @param qs_parent joint positon of parent node
        @param qs_config joint configuration is approved,
                otherwise None'''
        qs_config = self.clampJointLimits(qs_proposed)
        collision_free = self.checkEdgeCollision(qs_config, qs_parent)

        ee_pose = self.manip.ComputeFK(qs_config)
        ee_constraint = self.evaluateConstraints(ConstraintType.PATH_EE, pose=ee_pose)
        joint_constraint = self.evaluateConstraints(ConstraintType.PATH_JOINT, config=qs_config, pose=ee_pose)
        if collision_free and ee_constraint and joint_constraint:
            return qs_config
        else:
            return None

    def clampJointLimits(self, qs):
        '''Given a proposed next joint space location, check that it is within
        joint limits. If it is not, clamp it to the nearest joint limit
        (joint wise)
        @param qs Joint configuration
        @param qs_new Joint configuration'''
        (lower, upper) = self.manip.GetJointLimits()
        qs_new = [max(lower[i], min(qs[i], upper[i])) for i in range(len(lower))]
        return qs_new

    def checkEdgeCollision(self, q, q_parent):
        '''Check if path from q_first to q_second is collision free
        @param q Joint configuration
        @return collisionFree (boolean) True if collision free''' 
        # Reject if end point is not collision free
        if not self.manip.IsCollisionFree(q, obstacles=self.obstacles):
            return False
        cdist = util.cspaceLength([q_parent, q])
        count = int(cdist / 0.025) # Check every 0.025 distance (a little arbitrary)

        # linearly interpolate between that at some step size and check all those points
        interp = [numpy.linspace(q_parent[i], q[i], count+1).tolist() for i in range(len(q))]
        middle_qs = numpy.transpose(interp)[1:-1] # Remove given points
        return all((self.manip.IsCollisionFree(m, obstacles=self.obstacles) for m in middle_qs)) 

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
