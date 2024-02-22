import random
from collections import namedtuple
import pb_robot_spot.geometry as geometry

IKFastInfo = namedtuple('IKFastInfo', ['module_name', 'base_link', 'ee_link', 'free_joints'])
USE_ALL = False
USE_CURRENT = None

def compute_forward_kinematics(fk_fn, conf):
    pose = fk_fn(list(conf))
    pos, rot = pose
    quat = geometry.quat_from_matrix(rot) # [X,Y,Z,W]
    return pos, quat


def compute_inverse_kinematics(ik_fn, pose, sampled=[]):
    pos = geometry.point_from_pose(pose)
    rot = geometry.matrix_from_quat(geometry.quat_from_pose(pose)).tolist()
    if len(sampled) == 0:
        solutions = ik_fn(list(rot), list(pos))
    else:
        solutions = ik_fn(list(rot), list(pos), list(sampled))
    if solutions is None:
        return []
    return solutions


def get_ik_limits(robot, joint, limits=USE_ALL):
    if limits is USE_ALL:
        return robot.get_joint_limits(joint)
    elif limits is USE_CURRENT:
        value = robot.get_joint_position(joint)
        return value, value
    return limits


def select_solution(body, joints, solutions, nearby_conf=USE_ALL, **kwargs):
    if not solutions:
        return None
    if nearby_conf is USE_ALL:
        return random.choice(solutions)
    if nearby_conf is USE_CURRENT:
        nearby_conf = body.get_joint_positions(joints)
    # TODO: sort by distance before collision checking
    # TODO: search over neighborhood of sampled joints when nearby_conf != None
    return min(solutions, key=lambda conf: geometry.get_distance(nearby_conf, conf, **kwargs))
