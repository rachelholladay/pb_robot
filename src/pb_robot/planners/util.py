#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Various utility functions for constrained tool manipulation
'''

import numpy
import random

def generatePath(path_array):
    '''Given an array of poses, create an OpenRAVE executable path
    @param path_array Numpy array of joint poses
    @return traj OpenRave path'''

    '''
    if path_array is None or len(path_array) == 0:
        return None
    path = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    cspec = robot.GetActiveConfigurationSpecification('linear')
    path.Init(cspec)

    for i in xrange(len(path_array)):
        path.Insert(i, path_array[i])
    processed_path = robot.PostProcessPath(path) #, constrained=True, smooth=False)
    return processed_path
    '''
    if path_array is None:
        return None

    # Remove duplicate points
    removeRows = []
    for i in range(len(path_array)-1):
        diff = sum(numpy.subtract(path_array[i], path_array[i+1]))
        if abs(diff) < 1e-2:
            removeRows += [i]
    # Remove all rows after.
    simplifiedPath = numpy.delete(path_array, removeRows, 0)
    return simplifiedPath

def getDirectory():
    '''Get the file path for the location of kinbody
    @return object_path (string) Path to objects folder'''
    from catkin.find_in_workspaces import find_in_workspaces
    package_name = 'mcube_objects'
    directory = 'data'
    objects_path = find_in_workspaces(
        search_dirs=['share'],
        project=package_name,
        path=directory,
        first_match_only=True)
    if len(objects_path) == 0:
        raise RuntimeError('Can\'t find directory {}/{}'.format(
            package_name, directory))
    else:
        objects_path = objects_path[0]
    return objects_path

def SampleTSRForPose(tsr_chain):
    '''Shortcutting function for randomly samping a 
    pose from a tsr chain
    @param tsr_chain Chain to sample from
    @return ee_pose Pose from tsr chain'''
    tsr_idx = random.randint(0, len(tsr_chain)-1)
    sampled_tsr = tsr_chain[tsr_idx]
    ee_pose = sampled_tsr.sample()
    return ee_pose

def CreateTSRFromPose(manip, pose):
    '''Create a TSR that, when sampled, produces one pose. This 
    simply creates a common interface for the planner
    @param manip Manipulator to use use (required for tsr)
    @param pose 4x4 transform to center TSR on
    @return tsr_chain chain with single pose TSR'''
    import tsr
    goal_tsr = tsr.tsr.TSR(T0_w=pose)
    tsr_chain = tsr.tsr.TSRChain(sample_goal=True, TSR=goal_tsr)
    return tsr_chain

def cspaceLength(path):
    '''Compute the euclidean distance in joint space of a path by summing along pts
    @param path List of joint configurations 
    @return total_dist Scaler sum of distances'''
    total_dist = 0
    for i in range(len(path)-1):
        total_dist += numpy.linalg.norm(numpy.subtract(path[i+1], path[i]))
    return total_dist

