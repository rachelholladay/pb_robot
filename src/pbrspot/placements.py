import numpy as np
import pbrspot

PI = np.pi
CIRCULAR_LIMITS = -PI, PI

def stable_z_on_aabb(body, aabb):
    center, extent = pbrspot.aabb.get_center_extent(body)
    _, upper = aabb
    return (upper + extent/2 + (body.get_point() - center))[2]

def stable_z(body, surface, surface_link=None):
    return stable_z_on_aabb(body, pbrspot.aabb.get_aabb(surface, link=surface_link))

def is_placed_on_aabb(body, bottom_aabb, above_epsilon=1e-2, below_epsilon=0.0):
    top_aabb = pbrspot.aabb.get_aabb(body) # TODO: approximate_as_prism
    bottom_z_max = bottom_aabb[1][2]
    top_z_min = top_aabb[0][2]
    return ((bottom_z_max - below_epsilon) <= top_z_min) and \
           (top_z_min <= (bottom_z_max + above_epsilon)) and \
           (pbrspot.aabb.aabb_contains_aabb(pbrspot.aabb.aabb2d_from_aabb(top_aabb), pbrspot.aabb.aabb2d_from_aabb(bottom_aabb)))

def is_placement(body, surface, **kwargs):
    return is_placed_on_aabb(body, pbrspot.aabb.get_aabb(surface), **kwargs)

def sample_placement_on_aabb(top_body, bottom_aabb, top_pose=pbrspot.geometry.unit_pose(),
                             percent=1.0, max_attempts=50, epsilon=1e-3):
    # TODO: transform into the coordinate system of the bottom
    # TODO: maybe I should instead just require that already in correct frame
    start_pose = top_body.get_pose()
    for _ in range(max_attempts):
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        rotation = pbrspot.geometry.Euler(yaw=theta)
        top_body.set_pose(pbrspot.geometry.multiply(pbrspot.geometry.Pose(euler=rotation), top_pose))
        center, extent = pbrspot.aabb.get_center_extent(top_body)
        lower = (np.array(bottom_aabb[0]) + percent*extent/2)[:2]
        upper = (np.array(bottom_aabb[1]) - percent*extent/2)[:2]
        if np.less(upper, lower).any():
            continue
        x, y = np.random.uniform(lower, upper)
        z = (bottom_aabb[1] + extent/2.)[2] + epsilon
        point = np.array([x, y, z]) + (top_body.get_point() - center)
        pose = pbrspot.geometry.multiply(pbrspot.geometry.Pose(point, rotation), top_pose)
        top_body.set_pose(start_pose)
        return pose
    top_body.set_pose(start_pose)
    return None

def sample_placement(top_body, bottom_body, bottom_link=None, **kwargs):
    bottom_aabb = pbrspot.aabb.get_aabb(bottom_body, link=bottom_link)
    return sample_placement_on_aabb(top_body, bottom_aabb, **kwargs)

def is_center_on_aabb(body, bottom_aabb, above_epsilon=1e-2, below_epsilon=0.0):
    # TODO: compute AABB in origin
    # TODO: use center of mass?
    assert (0 <= above_epsilon) and (0 <= below_epsilon)
    center, extent = pbrspot.aabb.get_center_extent(body) # TODO: approximate_as_prism
    base_center = center - np.array([0, 0, extent[2]])/2
    top_z_min = base_center[2]
    bottom_z_max = bottom_aabb[1][2]
    return ((bottom_z_max - abs(below_epsilon)) <= top_z_min <= (bottom_z_max + abs(above_epsilon))) and \
           (pbrspot.aabb.aabb_contains_point(base_center[:2], pbrspot.aabb.aabb2d_from_aabb(bottom_aabb)))

def is_center_stable(body, surface, **kwargs):
    return is_center_on_aabb(body, pbrspot.aabb.get_aabb(surface), **kwargs)
