import importlib
import time
import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))

from itertools import islice
import pb_robot_spot
import pb_robot_spot.geometry as geometry
import pb_robot_spot.planning as planning
import pb_robot_spot.helper as helper

from .utils import compute_inverse_kinematics

INF = np.inf

def import_ikfast(ikfast_info):
    # https://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path
    #print(sys.modules['__main__'].__file__)
    #return importlib.import_module('pybullet_tools.ikfast.{}'.format(ikfast_info.module_name), package=None)
    #return importlib.import_module('{}'.format(ikfast_info.module_name), package='pybullet_tools.ikfast')
    return importlib.import_module('ikfast.{}'.format(ikfast_info.module_name), package=None)


def is_ik_compiled(ikfast_info):
    try:
        import_ikfast(ikfast_info)
        return True
    except ImportError:
        return False


def get_base_from_ee(robot, ikfast_info, tool_link, world_from_target):
    
    world_from_base = (robot.link_from_name(ikfast_info.base_link)).get_link_pose()
    world_from_ee = (robot.link_from_name(ikfast_info.ee_link)).get_link_pose()
    world_from_tool = tool_link.get_link_pose()
    tool_from_ee = geometry.multiply(geometry.invert(world_from_tool), world_from_ee)
    base_from_ee = geometry.multiply(geometry.invert(world_from_base), world_from_target, tool_from_ee)
    return base_from_ee


def get_ordered_ancestors(robot, link):
    #return prune_fixed_joints(robot, get_link_ancestors(robot, link)[1:] + [link])
    return link.get_link_ancestors()[1:] + [link]


def get_ik_joints(robot, ikfast_info, tool_link):
    # Get joints between base and ee
    # Ensure no joints between ee and tool
    base_link = robot.link_from_name(ikfast_info.base_link)
    ee_link = robot.link_from_name(ikfast_info.ee_link)
    ee_ancestors = get_ordered_ancestors(robot, ee_link)
    tool_ancestors = get_ordered_ancestors(robot, tool_link)

    [first_joint] = [robot.parent_joint_from_link(link) for link in tool_ancestors 
                     if (robot.parent_joint_from_link(link)).get_link_parent().linkID == base_link.linkID]

    # Leverage that linkID and jointID match to convert from links to their relevant joints
    j_ee_ancestors = [e.linkID for e in ee_ancestors]
    j_tool_ancestors = [e.linkID for e in tool_ancestors]

    #assert robot.prune_fixed_joints(ee_ancestors) == robot.prune_fixed_joints(tool_ancestors)
    assert robot.prune_fixed_joints(j_ee_ancestors) == robot.prune_fixed_joints(j_tool_ancestors)
    ##assert base_link in ee_ancestors # base_link might be -1
    #ik_joints = robot.prune_fixed_joints(ee_ancestors[ee_ancestors.index(first_joint):])
    ik_joints = robot.prune_fixed_joints(j_ee_ancestors[ee_ancestors.index(first_joint):])
    free_joints = robot.joints_from_names(ikfast_info.free_joints)
    assert set(free_joints) <= set(ik_joints)
    assert len(ik_joints) == 6 + len(free_joints)
    return ik_joints


def ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
                              fixed_joints=[], max_attempts=INF, max_time=INF,
                              norm=INF, max_distance=INF, **kwargs):
    assert (max_attempts < INF) or (max_time < INF)
    if max_distance is None:
        max_distance = INF
    #assert is_ik_compiled(ikfast_info)
    ikfast = import_ikfast(ikfast_info)
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link) 
    free_joints = list(robot.joints_from_names(ikfast_info.free_joints))
    base_from_ee = get_base_from_ee(robot, ikfast_info, tool_link, world_from_target)
    difference_fn = planning.get_difference_fn(robot, ik_joints)
    current_conf = robot.get_joint_positions(ik_joints)
    current_positions = robot.get_joint_positions(free_joints)

    # TODO: handle circular joints
    free_deltas = np.array([0. if joint in fixed_joints else max_distance for joint in free_joints])
    lower_limits = np.maximum(robot.get_min_limits(free_joints), current_positions - free_deltas)
    upper_limits = np.minimum(robot.get_max_limits(free_joints), current_positions + free_deltas)
    generator = planning.interval_generator(lower_limits, upper_limits) 
    if max_attempts < INF:
        generator = islice(generator, max_attempts)
    start_time = time.time()
    for free_positions in generator:
        if max_time < pb_robot_spot.utils.elapsed_time(start_time):
            break
        for conf in helper.randomize(compute_inverse_kinematics(ikfast.get_ik, base_from_ee, free_positions)): 
            difference = difference_fn(current_conf, conf) 
            if not robot.violates_limits(ik_joints, conf) and geometry.get_length(difference, norm=norm): 
                #set_joint_positions(robot, ik_joints, conf)
                yield conf


def closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
                               max_candidates=INF, norm=INF, **kwargs):
    start_time = time.time()
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    current_conf = robot.get_joint_positions(ik_joints)
    generator = ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, norm=norm, **kwargs)
    if max_candidates < INF:
        generator = islice(generator, max_candidates) 
    solutions = list(generator)
    #print('Identified {} IK solutions in {:.3f} seconds'.format(len(solutions), elapsed_time(start_time)))
    # TODO: relative to joint limits
    difference_fn = planning.get_difference_fn(robot, ik_joints)
    #distance_fn = get_distance_fn(robot, ik_joints)
    #set_joint_positions(robot, ik_joints, closest_conf)
    return iter(sorted(solutions, key=lambda q: geometry.get_length(difference_fn(q, current_conf), norm=norm)))
