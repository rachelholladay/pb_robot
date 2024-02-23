from __future__ import print_function
import math
import numpy as np
import pybullet as p
import pbrspot.transformations as transformations
import pbrspot.helper as helper 

CLIENT = 0
# Geometry

#Pose = namedtuple('Pose', ['position', 'orientation'])

def Point(x=0., y=0., z=0.):
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler))

#def Pose2d(x=0., y=0., yaw=0.):
#    return np.array([x, y, yaw])

def invert(pose):
    (point, quat) = pose
    return p.invertTransform(point, quat)

def multiply(*poses):
    pose = poses[0]
    for next_pose in poses[1:]:
        pose = p.multiplyTransforms(pose[0], pose[1], *next_pose)
    return pose

def invert_quat(quat):
    pose = (unit_point(), quat)
    return quat_from_pose(invert(pose))

def multiply_quats(*quats):
    return quat_from_pose(multiply(*[(unit_point(), quat) for quat in quats]))

def unit_from_theta(theta):
    return np.array([np.cos(theta), np.sin(theta)])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)

def unit_point():
    return (0., 0., 0.)

def unit_quat():
    return quat_from_euler([0, 0, 0]) # [X,Y,Z,W]

def quat_from_axis_angle(axis, angle): # axis-angle
    #return get_unit_vector(np.append(vec, [angle]))
    return np.append(math.sin(angle/2) * get_unit_vector(axis), [math.cos(angle / 2)])

def unit_pose():
    return (unit_point(), unit_quat())

def get_length(vec, norm=2):
    return np.linalg.norm(vec, ord=norm)

def get_difference(p1, p2):
    return np.array(p2) - np.array(p1)

def get_distance(p1, p2, **kwargs):
    return get_length(get_difference(p1, p2), **kwargs)

def angle_between(vec1, vec2):
    return np.math.acos(np.dot(vec1, vec2) / (get_length(vec1) *  get_length(vec2)))

def get_angle(q1, q2):
    dx, dy = np.array(q2[:2]) - np.array(q1[:2])
    return np.math.atan2(dy, dx)

def get_unit_vector(vec):
    norm = get_length(vec)
    if norm == 0:
        return vec
    return np.array(vec) / norm

def z_rotation(theta):
    return quat_from_euler([0, 0, theta])

def quat_from_matrix(mat):
    matrix = np.eye(4)
    matrix[:3, :3] = mat
    return transformations.quaternion_from_matrix(matrix)

def point_from_tform(tform):
    return np.array(tform)[:3, 3]

def matrix_from_tform(tform):
    return np.array(tform)[:3, :3]

def point_from_pose(pose):
    return pose[0]

def quat_from_pose(pose):
    return pose[1]

def pose_from_tform(tform):
    return point_from_tform(tform), quat_from_matrix(matrix_from_tform(tform))

def wrap_angle(theta, lower=-np.pi): # [-np.pi, np.pi)
    return (theta - lower) % (2 * np.pi) + lower

def tform_from_pose(pose):
    (point, quat) = pose
    tform = np.eye(4)
    tform[:3, 3] = point
    tform[:3, :3] = matrix_from_quat(quat)
    return tform

def tform_from_point(point):
    tform = np.eye(4)
    tform[0:3, 3] = point
    return tform

def matrix_from_quat(quat):
    return np.array(p.getMatrixFromQuaternion(quat, physicsClientId=CLIENT)).reshape(3, 3)

def circular_difference(theta2, theta1):
    return wrap_angle(theta2 - theta1)

def base_values_from_pose(pose, tolerance=1e-3):
    (point, quat) = pose
    x, y, _ = point
    roll, pitch, yaw = euler_from_quat(quat)
    assert (abs(roll) < tolerance) and (abs(pitch) < tolerance)
    return (x, y, yaw)
pose2d_from_pose = base_values_from_pose

def pose_from_pose2d(pose2d):
    x, y, theta = pose2d
    return Pose(Point(x=x, y=y), Euler(yaw=theta))

def pose_from_base_values(base_values, default_pose=unit_pose()):
    x, y, yaw = base_values
    _, _, z = point_from_pose(default_pose)
    roll, pitch, _ = euler_from_quat(quat_from_pose(default_pose))
    return (x, y, z), quat_from_euler([roll, pitch, yaw])

def quat_angle_between(quat0, quat1): # quaternion_slerp
    #p.computeViewMatrixFromYawPitchRoll()
    q0 = transformations.unit_vector(quat0[:4])
    q1 = transformations.unit_vector(quat1[:4])
    d = helper.clip(np.dot(q0, q1), min_value=-1., max_value=+1.)
    angle = math.acos(d)
    # TODO: angle_between
    #delta = p.getDifferenceQuaternion(quat0, quat1)
    #angle = math.acos(delta[-1])
    return angle

def quatToAxisAngle(quat):
    # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
    # Except its reverse order (i.e. angle at the end)
    angle = 2 * math.acos(quat[3])

    # Check if angle is zero. If yes, axis can be any normalized value
    if abs(angle) < 0.001:  
        x = 1.0
        y = 0.0
        z = 0.0
    else:
        x = quat[0] / math.sqrt(1 - (quat[3]*quat[3]))
        y = quat[1] / math.sqrt(1 - (quat[3]*quat[3]))
        z = quat[2] / math.sqrt(1 - (quat[3]*quat[3]))
    return angle, np.array([x, y, z])

def get_position_waypoints(start_point, direction, quat, step_size=0.01):
    distance = get_length(direction)
    unit_direction = get_unit_vector(direction)
    for t in np.arange(0, distance, step_size):
        point = start_point + t*unit_direction
        yield (point, quat)
    yield (start_point + direction, quat)

def get_quaternion_waypoints(point, start_quat, end_quat, step_size=np.pi/16):
    angle = quat_angle_between(start_quat, end_quat)
    for t in np.arange(0, angle, step_size):
        fraction = t/angle
        quat = p.getQuaternionSlerp(start_quat, end_quat, interpolationFraction=fraction)
        #quat = quaternion_slerp(start_quat, end_quat, fraction=fraction)
        yield (point, quat)
    yield (point, end_quat)

def interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
    pos1, quat1 = pose1
    pos2, quat2 = pose2
    num_steps = int(math.ceil(max(get_distance(pos1, pos2)/pos_step_size,
                                  quat_angle_between(quat1, quat2)/ori_step_size)))
    for i in range(num_steps):
        fraction = float(i) / num_steps
        pos = (1-fraction)*np.array(pos1) + fraction*np.array(pos2)
        quat = p.getQuaternionSlerp(quat1, quat2, interpolationFraction=fraction)
        #quat = quaternion_slerp(quat1, quat2, fraction=fraction)
        yield (pos, quat)
    yield pose2

# Polygonal surfaces

def create_rectangular_surface(width, length):
    # TODO: unify with rectangular_mesh
    extents = np.array([width, length, 0]) / 2.
    unit_corners = [(-1, -1), (+1, -1), (+1, +1), (-1, +1)]
    return [np.append(c, 0) * extents for c in unit_corners]

def is_point_in_polygon(point, polygon):
    sign = None
    for i in range(len(polygon)):
        v1, v2 = np.array(polygon[i - 1][:2]), np.array(polygon[i][:2])
        delta = v2 - v1
        normal = np.array([-delta[1], delta[0]])
        dist = normal.dot(point[:2] - v1)
        if i == 0:  # TODO: equality?
            sign = np.sign(dist)
        elif np.sign(dist) != sign:
            return False
    return True

def distance_from_segment(x1, y1, x2, y2, x3, y3): # x3,y3 is the point
    # https://stackoverflow.com/questions/10983872/distance-from-a-point-to-a-polygon
    # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    px = x2 - x1
    py = y2 - y1
    norm = px*px + py*py
    u = ((x3 - x1) * px + (y3 - y1) * py) / float(norm)
    if u > 1:
        u = 1
    elif u < 0:
        u = 0
    x = x1 + u * px
    y = y1 + u * py
    dx = x - x3
    dy = y - y3
    return math.sqrt(dx*dx + dy*dy)

def tform_point(affine, point):
    return point_from_pose(multiply(affine, Pose(point=point)))

def apply_affine(affine, points):
    return [tform_point(affine, po) for po in points]

def is_mesh_on_surface(polygon, world_from_surface, mesh, world_from_mesh, epsilon=1e-2):
    surface_from_mesh = multiply(invert(world_from_surface), world_from_mesh)
    points_surface = apply_affine(surface_from_mesh, mesh.vertices)
    min_z = np.min(points_surface[:, 2])
    return (abs(min_z) < epsilon) and \
           all(is_point_in_polygon(p, polygon) for p in points_surface)

def is_point_on_surface(polygon, world_from_surface, point_world):
    [point_surface] = apply_affine(invert(world_from_surface), [point_world])
    return is_point_in_polygon(point_surface, polygon[::-1])

def sample_polygon_tform(polygon, points):
    min_z = np.min(points[:, 2])
    aabb_min = np.min(polygon, axis=0)
    aabb_max = np.max(polygon, axis=0)
    while True:
        x = np.random.uniform(aabb_min[0], aabb_max[0])
        y = np.random.uniform(aabb_min[1], aabb_max[1])
        theta = np.random.uniform(0, 2 * np.pi)
        point = Point(x, y, -min_z)
        quat = Euler(yaw=theta)
        surface_from_origin = Pose(point, quat)
        yield surface_from_origin
        # if all(is_point_in_polygon(p, polygon) for p in apply_affine(surface_from_origin, points)):
        #  yield surface_from_origin

def sample_surface_pose(polygon, world_from_surface, mesh):
    for surface_from_origin in sample_polygon_tform(polygon, mesh.vertices):
        world_from_mesh = multiply(world_from_surface, surface_from_origin)
        if is_mesh_on_surface(polygon, world_from_surface, mesh, world_from_mesh):
            yield world_from_mesh

#####################################

# Convex Hulls

def convex_hull(points):
    from scipy.spatial import ConvexHull
    # TODO: cKDTree is faster, but KDTree can do all pairs closest
    hull = ConvexHull(points)
    new_indices = {i: ni for ni, i in enumerate(hull.vertices)}
    vertices = hull.points[hull.vertices, :]
    faces = np.vectorize(lambda i: new_indices[i])(hull.simplices)
    return helper.Mesh(vertices.tolist(), faces.tolist())

def convex_signed_area(vertices):
    if len(vertices) < 3:
        return 0.
    vertices = [np.array(v[:2]) for v in vertices]
    segments = helper.safe_zip(vertices, vertices[1:] + vertices[:1])
    return sum(np.cross(v1, v2) for v1, v2 in segments) / 2.

def convex_area(vertices):
    return abs(convex_signed_area(vertices))

def convex_centroid(vertices):
    # TODO: also applies to non-overlapping polygons
    vertices = [np.array(v[:2]) for v in vertices]
    segments = list(helper.safe_zip(vertices, vertices[1:] + vertices[:1]))
    return sum((v1 + v2)*np.cross(v1, v2) for v1, v2 in segments) \
           / (6.*convex_signed_area(vertices))

def mesh_from_points(points):
    vertices, indices = convex_hull(points)
    new_indices = []
    for triplet in indices:
        centroid = np.average(vertices[triplet], axis=0)
        v1, v2, v3 = vertices[triplet]
        normal = np.cross(v3 - v1, v2 - v1)
        if normal.dot(centroid) > 0:
            # if normal.dot(centroid) < 0:
            triplet = triplet[::-1]
        new_indices.append(tuple(triplet))
    return helper.Mesh(vertices.tolist(), new_indices)

def rectangular_mesh(width, length):
    # TODO: 2.5d polygon
    extents = np.array([width, length, 0])/2.
    unit_corners = [(-1, -1), (+1, -1), (+1, +1), (-1, +1)]
    vertices = [np.append(c, [0])*extents for c in unit_corners]
    faces = [(0, 1, 2), (2, 3, 0)]
    return helper.Mesh(vertices, faces)

def tform_mesh(affine, mesh):
    return helper.Mesh(apply_affine(affine, mesh.vertices), mesh.faces)

def grow_polygon(vertices, radius, n=8):
    vertices2d = [vertex[:2] for vertex in vertices]
    if not vertices2d:
        return []
    points = []
    for vertex in convex_hull(vertices2d).vertices:
        points.append(vertex)
        if radius > 0:
            for theta in np.linspace(0, 2*np.pi, num=n, endpoint=False):
                points.append(vertex + radius*unit_from_theta(theta))
    return convex_hull(points).vertices



#### Distance metrics
def GeodesicError(t1, t2):
    """
    Computes the error in global coordinates between two transforms.
    @param t1 current transform
    @param t2 goal transform
    @return a 4-vector of [dx, dy, dz, solid angle]
    """
    trel = np.dot(np.linalg.inv(t1), t2)
    trans = np.dot(t1[0:3, 0:3], trel[0:3, 3])
    #omega = openravepy.axisAngleFromRotationMatrix(trel[0:3, 0:3]) 
    # XXX This is hack for now. Sometimes we get an improper rotation matrix
    # with a determinant of -1 instead of 1. We negate it to get a 
    # proper rotation matrix but more investigation is needed.
    try:
        omega, _, _ = transformations.rotation_from_matrix(trel)
    except:
        omega, _, _ = transformations.rotation_from_matrix(-trel)
    angle = np.linalg.norm(omega)
    return np.hstack((trans, angle))

def GeodesicDistance(t1, t2, r=1.0):
    """
    Computes the geodesic distance between two transforms
    @param t1 current transform
    @param t2 goal transform
    @param r in units of meters/radians converts radians to meters
    """
    error = GeodesicError(t1, t2)
    error[3] = r * error[3]
    return np.linalg.norm(error)

