import random
import time
from itertools import product, combinations
from collections import namedtuple
from .crg_planners.rrt_connect import birrt, direct_path

import numpy as np
import pybullet as p
import pb_robot_spot
import pb_robot_spot.geometry as geometry

PI = np.pi
CIRCULAR_LIMITS = -PI, PI
CLIENT = 0
MAX_DISTANCE = 0
NullSpace = namedtuple('Nullspace', ['lower', 'upper', 'range', 'rest'])

# Joint motion planning

def uniform_generator(d):
    while True:
        yield np.random.uniform(size=d)

def halton_generator(d):
    import ghalton
    # sequencer = ghalton.Halton(d)
    sequencer = ghalton.GeneralizedHalton(d, random.randint(0, 1000))
    while True:
        yield sequencer.get(1)[0]

def unit_generator(d, use_halton=False):
    return halton_generator(d) if use_halton else uniform_generator(d)

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    generator = unit_generator(len(joints), **kwargs)
    lower_limits, upper_limits = body.get_custom_limits(joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    limits_extents = np.array(upper_limits) - np.array(lower_limits)
    def fn():
        return tuple(next(generator) * limits_extents + np.array(lower_limits))
    return fn

def interval_generator(lower, upper, **kwargs):
    assert len(lower) == len(upper)
    assert np.less_equal(lower, upper).all()
    if np.equal(lower, upper).all():
        return iter([lower])
    return (weights*lower + (1-weights)*upper for weights in unit_generator(d=len(lower), **kwargs))

def get_halton_sample_fn(body, joints, **kwargs):
    return get_sample_fn(body, joints, use_halton=True, **kwargs)

def get_difference_fn(body, joints):
    circular_joints = [joint.is_circular() for joint in joints]

    def fn(q2, q1):
        return tuple(geometry.circular_difference(value2, value1) if circular else (value2 - value1)
                     for circular, value2, value1 in zip(circular_joints, q2, q1))
    return fn

def get_distance_fn(body, joints, weights=None): #, norm=2):
    # TODO: use the energy resulting from the mass matrix here?
    if weights is None:
        weights = 1*np.ones(len(joints)) # TODO: use velocities here
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
        #return np.linalg.norm(np.multiply(weights * diff), ord=norm)
    return fn

def get_refine_fn(body, joints, num_steps=0):
    difference_fn = get_difference_fn(body, joints)
    num_steps = num_steps + 1
    def fn(q1, q2):
        q = q1
        for i in range(num_steps):
            positions = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            q = tuple(positions)
            #q = tuple(wrap_positions(body, joints, positions))
            yield q
    return fn

def refine_path(body, joints, waypoints, num_steps):
    refine_fn = get_refine_fn(body, joints, num_steps)
    refined_path = []
    for v1, v2 in zip(waypoints, waypoints[1:]):
        refined_path += list(refine_fn(v1, v2))
    return refined_path

DEFAULT_RESOLUTION = 0.05

def get_extend_fn(body, joints, resolutions=None, norm=2):
    # norm = 1, 2, INF
    if resolutions is None:
        resolutions = DEFAULT_RESOLUTION*np.ones(len(joints))
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        #steps = int(np.max(np.abs(np.divide(difference_fn(q2, q1), resolutions))))
        steps = int(np.linalg.norm(np.divide(difference_fn(q2, q1), resolutions), ord=norm))
        refine_fn = get_refine_fn(body, joints, num_steps=steps)
        return refine_fn(q1, q2)
    return fn

def waypoints_from_path(path, tolerance=1e-3):
    if len(path) < 2:
        return path

    def difference_fn(q2, q1):
        return np.array(q2) - np.array(q1)
    #difference_fn = get_difference_fn(body, joints)

    waypoints = [path[0]]
    last_conf = path[1]
    last_difference = geometry.get_unit_vector(difference_fn(last_conf, waypoints[-1]))
    for conf in path[2:]:
        difference = geometry.get_unit_vector(difference_fn(conf, waypoints[-1]))
        if not np.allclose(last_difference, difference, atol=tolerance, rtol=0):
            waypoints.append(last_conf)
            difference = geometry.get_unit_vector(difference_fn(conf, waypoints[-1]))
        last_conf = conf
        last_difference = difference
    waypoints.append(last_conf)
    return waypoints

def get_moving_links(body, joints):
    moving_links = set()
    for joint in joints:
        link = joint.child_link_from_joint()
        if link not in moving_links:
            moving_links.update(link.get_link_subtree())
    return list(moving_links)

def get_moving_pairs(body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = body.get_moving_links(moving_joints)
    for link1, link2 in combinations(moving_links, 2):
        ancestors1 = set(link1.get_joint_ancestors()) & set(moving_joints)
        ancestors2 = set(link2.get_joint_ancestors()) & set(moving_joints)
        if ancestors1 != ancestors2:
            yield link1, link2


def get_self_link_pairs(body, joints, disabled_collisions=set(), only_moving=True):
    moving_links = body.get_moving_links(joints)
    fixed_links = list(set(body.links) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if only_moving:
        check_link_pairs.extend(get_moving_pairs(body, joints))
    else:
        check_link_pairs.extend(combinations(moving_links, 2))
    check_link_pairs = list(filter(lambda pair: not pair.are_links_adjacent(), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                   (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs

def get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                     custom_limits={}, **kwargs):
    # TODO: convert most of these to keyword arguments
    check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) \
        if self_collisions else []
    moving_links = frozenset(get_moving_links(body, joints))
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [(body, moving_links)] + attached_bodies
    #moving_bodies = [body] + [attachment.child for attachment in attachments]
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    lower_limits, upper_limits = body.get_custom_limits(joints, custom_limits)

    # TODO: maybe prune the link adjacent to the robot
    # TODO: test self collision with the holding
    def collision_fn(q):
        if not pb_robot_spot.helper.all_between(lower_limits, q, upper_limits):
            #print('Joint limits violated')
            return True
        body.set_joint_positions(joints, q)
        for attachment in attachments:
            attachment.assign()
        for link1, link2 in check_link_pairs:
            # Self-collisions should not have the max_distance parameter
            if pb_robot_spot.collisions.pairwise_link_collision(body, link1, body, link2): #, **kwargs):
                #print(get_body_name(body), get_link_name(body, link1), get_link_name(body, link2))
                return True
        for body1, body2 in check_body_pairs:
            if pb_robot_spot.collisions.pairwise_collision(body1, body2, **kwargs):
                #print(get_body_name(body1), get_body_name(body2))
                return True
        return False
    return collision_fn

def plan_waypoints_joint_motion(body, joints, waypoints, start_conf=None, obstacles=[], attachments=[],
                                self_collisions=True, disabled_collisions=set(),
                                resolutions=None, custom_limits={}, max_distance=MAX_DISTANCE):
    extend_fn = get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)
    if start_conf is None:
        start_conf = body.get_joint_positions(joints)
    else:
        assert len(start_conf) == len(joints)

    for i, waypoint in enumerate([start_conf] + list(waypoints)):
        if collision_fn(waypoint):
            #print("Warning: waypoint configuration {}/{} is in collision".format(i, len(waypoints)))
            return None
    path = [start_conf]
    for waypoint in waypoints:
        assert len(joints) == len(waypoint)
        for q in extend_fn(path[-1], waypoint):
            if collision_fn(q):
                return None
            path.append(q)
    return path

def plan_direct_joint_motion(body, joints, end_conf, **kwargs):
    return plan_waypoints_joint_motion(body, joints, [end_conf], **kwargs)

def check_initial_end(start_conf, end_conf, collision_fn):
    if collision_fn(start_conf):
        print("Warning: initial configuration is in collision")
        return False
    if collision_fn(end_conf):
        print("Warning: end configuration is in collision")
        return False
    return True

def plan_joint_motion(body, joints, end_conf, obstacles=[], attachments=[],
                      self_collisions=True, disabled_collisions=set(),
                      weights=None, resolutions=None, max_distance=MAX_DISTANCE, custom_limits={}, **kwargs):

    assert len(joints) == len(end_conf)
    sample_fn = get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = get_distance_fn(body, joints, weights=weights)
    extend_fn = get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)

    start_conf = body.get_joint_positions(joints)

    if not check_initial_end(start_conf, end_conf, collision_fn):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)
    #return plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn)

def plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn, **kwargs):
    # TODO: cost metric based on total robot movement (encouraging greater distances possibly)
    from motion_planners.lazy_prm import lazy_prm
    path, samples, edges, colliding_vertices, colliding_edges = lazy_prm(
        start_conf, end_conf, sample_fn, extend_fn, collision_fn, num_samples=200, **kwargs)
    if path is None:
        return path

    #lower, upper = get_custom_limits(body, joints, circular_limits=CIRCULAR_LIMITS)
    def draw_fn(q): # TODO: draw edges instead of vertices
        return np.append(q[:2], [1e-3])
        #return np.array([1, 1, 0.25])*(q + np.array([0., 0., np.pi]))
    handles = []
    #for q1, q2 in zip(path, path[1:]): XXX comment back in?
    #    handles.append(add_line(draw_fn(q1), draw_fn(q2), color=(0, 1, 0)))
    for i1, i2 in edges:
        color = (0, 0, 1)
        if any(colliding_vertices.get(i, False) for i in (i1, i2)) or colliding_vertices.get((i1, i2), False):
            color = (1, 0, 0)
        elif not colliding_vertices.get((i1, i2), True):
            color = (0, 0, 0)
        #handles.append(add_line(draw_fn(samples[i1]), draw_fn(samples[i2]), color=color)) XXX comment back in?
    raw_input("")
    return path

#####################################

def get_closest_angle_fn(body, joints, reversible=True):
    assert len(joints) == 3
    linear_extend_fn = get_distance_fn(body, joints[:2])
    angular_extend_fn = get_distance_fn(body, joints[2:])

    def closest_fn(q1, q2):
        angle_and_distance = []
        for direction in [0, PI] if reversible else [PI]:
            angle = geometry.get_angle(q1[:2], q2[:2]) + direction
            distance = angular_extend_fn(q1[2:], [angle]) \
                       + linear_extend_fn(q1[:2], q2[:2]) \
                       + angular_extend_fn([angle], q2[2:])
            angle_and_distance.append((angle, distance))
        return min(angle_and_distance, key=lambda pair: pair[1])
    return closest_fn

def get_nonholonomic_distance_fn(body, joints, weights=None, **kwargs):
    assert weights is None
    closest_angle_fn = get_closest_angle_fn(body, joints, **kwargs)

    def distance_fn(q1, q2):
        _, distance = closest_angle_fn(q1, q2)
        return distance
    return distance_fn

def get_nonholonomic_extend_fn(body, joints, resolutions=None, **kwargs):
    assert resolutions is None
    assert len(joints) == 3
    linear_extend_fn = get_extend_fn(body, joints[:2])
    angular_extend_fn = get_extend_fn(body, joints[2:])
    closest_angle_fn = get_closest_angle_fn(body, joints, **kwargs)

    def extend_fn(q1, q2):
        angle, _ = closest_angle_fn(q1, q2)
        path = []
        for aq in angular_extend_fn(q1[2:], [angle]):
            path.append(np.append(q1[:2], aq))
        for lq in linear_extend_fn(q1[:2], q2[:2]):
            path.append(np.append(lq, [angle]))
        for aq in angular_extend_fn([angle], q2[2:]):
            path.append(np.append(q2[:2], aq))
        return path
    return extend_fn

def plan_nonholonomic_motion(body, joints, end_conf, obstacles=[], attachments=[],
                             self_collisions=True, disabled_collisions=set(),
                             weights=None, resolutions=None, reversible=True,
                             max_distance=MAX_DISTANCE, custom_limits={}, **kwargs):

    assert len(joints) == len(end_conf)
    sample_fn = get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = get_nonholonomic_distance_fn(body, joints, weights=weights, reversible=reversible)
    extend_fn = get_nonholonomic_extend_fn(body, joints, resolutions=resolutions, reversible=reversible)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments,
                                    self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)

    start_conf = body.get_joint_positions(joints)
    if not check_initial_end(start_conf, end_conf, collision_fn):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)

#####################################

# SE(2) pose motion planning

def get_base_difference_fn():
    def fn(q2, q1):
        dx, dy = np.array(q2[:2]) - np.array(q1[:2])
        dtheta = geometry.circular_difference(q2[2], q1[2])
        return (dx, dy, dtheta)
    return fn

def get_base_distance_fn(weights=1*np.ones(3)):
    difference_fn = get_base_difference_fn()
    def fn(q1, q2):
        difference = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, difference * difference))
    return fn

def plan_base_motion(body, end_conf, base_limits, obstacles=[], direct=False,
                     weights=1*np.ones(3), resolutions=0.05*np.ones(3),
                     max_distance=MAX_DISTANCE, **kwargs):
    def sample_fn():
        x, y = np.random.uniform(*base_limits)
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        return (x, y, theta)


    difference_fn = get_base_difference_fn()
    distance_fn = get_base_distance_fn(weights=weights)

    def extend_fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        n = int(np.max(steps)) + 1
        q = q1
        for i in range(n):
            q = tuple((1. / (n - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints

    def collision_fn(q):
        # TODO: update this function
        body.set_base_values(q)
        return any(pb_robot_spot.collisions.pairwise_collision(body, obs, max_distance=max_distance) for obs in obstacles)

    start_conf = body.get_base_values()
    if collision_fn(start_conf):
        print("Warning: initial configuration is in collision")
        return None
    if collision_fn(end_conf):
        print("Warning: end configuration is in collision")
        return None
    if direct:
        return direct_path(start_conf, end_conf, extend_fn, collision_fn)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)

def compute_jacobian(robot, link, positions=None):
    joints = robot.get_movable_joints()
    if positions is None:
        positions = robot.get_joint_positions(joints)
    assert len(joints) == len(positions)
    velocities = [0.0] * len(positions)
    accelerations = [0.0] * len(positions)
    translate, rotate = p.calculateJacobian(robot.id, link.linkID, geometry.unit_point(), positions,
                                            velocities, accelerations, physicsClientId=CLIENT)
    #movable_from_joints(robot, joints)
    return list(zip(*translate)), list(zip(*rotate)) # len(joints) x 3


def compute_joint_weights(robot, num=100):
    # http://openrave.org/docs/0.6.6/_modules/openravepy/databases/linkstatistics/#LinkStatisticsModel
    start_time = time.time()
    joints = robot.get_movable_joints()
    sample_fn = get_sample_fn(robot, joints)
    weighted_jacobian = np.zeros(len(joints))
    links = list(robot.links)
    # links = {l for j in joints for l in get_link_descendants(self.robot, j)}
    masses = [robot.get_mass(link.linkID) for link in links]  # Volume, AABB volume
    total_mass = sum(masses)
    for _ in range(num):
        conf = sample_fn()
        for mass, link in zip(masses, links):
            translate, rotate = compute_jacobian(robot, link, conf)
            weighted_jacobian += np.array([mass * np.linalg.norm(vec) for vec in translate]) / total_mass
    weighted_jacobian /= num
    print(list(weighted_jacobian))
    print(time.time() - start_time)
    return weighted_jacobian

def get_null_space(robot, joints, custom_limits={}):
    rest_positions = robot.get_joint_positions(joints)
    lower, upper = robot.get_custom_limits(joints, custom_limits)
    lower = np.maximum(lower, -10*np.ones(len(joints)))
    upper = np.minimum(upper, +10*np.ones(len(joints)))
    joint_ranges = 10*np.ones(len(joints))
    return NullSpace(list(lower), list(upper), list(joint_ranges), list(rest_positions))

def plan_cartesian_motion(robot, first_joint, target_link, waypoint_poses,
                          max_iterations=200, custom_limits={}, **kwargs):
    # TODO: fix stationary joints
    # TODO: pass in set of movable joints and take least common ancestor
    # TODO: update with most recent bullet updates
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
    # TODO: plan a path without needing to following intermediate waypoints

    lower_limits, upper_limits = robot.get_custom_limits(robot.get_movable_joints(), custom_limits)
    selected_links = robot.get_link_subtree(first_joint) # TODO: child_link_from_joint?
    selected_movable_joints = robot.prune_fixed_joints(selected_links)
    assert(target_link in selected_links)
    selected_target_link = selected_links.index(target_link)
    sub_robot = pb_robot_spot.utils.clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    sub_movable_joints = sub_robot.get_movable_joints(robot)
    #null_space = get_null_space(robot, selected_movable_joints, custom_limits=custom_limits)
    null_space = None

    solutions = []
    for target_pose in waypoint_poses:
        for iteration in range(max_iterations):
            sub_kinematic_conf = pb_robot_spot.utils.inverse_kinematics_helper(sub_robot, selected_target_link, target_pose, null_space=null_space)
            if sub_kinematic_conf is None:
                sub_robot.remove_body()
                return None
            sub_robot.set_joint_positions(sub_movable_joints, sub_kinematic_conf)
            if pb_robot_spot.utils.is_pose_close(sub_robot.get_link_pose(selected_target_link), target_pose, **kwargs):
                robot.set_joint_positions(selected_movable_joints, sub_kinematic_conf)
                kinematic_conf = robot.get_configuration()
                if not pb_robot_spot.helper.all_between(lower_limits, kinematic_conf, upper_limits):
                    #movable_joints = get_movable_joints(robot)
                    #print([(get_joint_name(robot, j), l, v, u) for j, l, v, u in
                    #       zip(movable_joints, lower_limits, kinematic_conf, upper_limits) if not (l <= v <= u)])
                    #print("Limits violated")
                    #wait_for_user()
                    sub_robot.remove_body()
                    return None
                #print("IK iterations:", iteration)
                solutions.append(kinematic_conf)
                break
        else:
            sub_robot.remove_body()
            return None
    sub_robot.remove_body()
    return solutions

def sub_inverse_kinematics(robot, first_joint, target_link, target_pose, **kwargs):
    solutions = plan_cartesian_motion(robot, first_joint, target_link, [target_pose], **kwargs)
    if solutions:
        return solutions[0]
    return None


# Reachability

def sample_reachable_base(robot, point, reachable_range=(0.25, 1.0)):
    radius = np.random.uniform(*reachable_range)
    x, y = radius*geometry.unit_from_theta(np.random.uniform(-np.pi, np.pi)) + point[:2]
    yaw = np.random.uniform(*CIRCULAR_LIMITS)
    base_values = (x, y, yaw)
    #set_base_values(robot, base_values)
    return base_values

def uniform_pose_generator(robot, gripper_pose, **kwargs):
    point = geometry.point_from_pose(gripper_pose)
    while True:
        base_values = sample_reachable_base(robot, point, **kwargs)
        if base_values is None:
            break
        yield base_values
