from collections import defaultdict, deque, namedtuple
import itertools
import pybullet as p
import pb_robot.utils_noBase as utils
import pb_robot.geometry as geometry
from .utils_noBase import CLIENT
from .joint import Joint
from .link import Link

JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute', # 0
    p.JOINT_PRISMATIC: 'prismatic', # 1
    p.JOINT_SPHERICAL: 'spherical', # 2
    p.JOINT_PLANAR: 'planar', # 3
    p.JOINT_FIXED: 'fixed', # 4
    p.JOINT_POINT2POINT: 'point2point', # 5
    p.JOINT_GEAR: 'gear', # 6
}

def createBody(rel_path=None, abs_path=None, pose=None, **kwargs):
    body_id = utils.load_model(rel_path=rel_path, abs_path=abs_path, pose=pose, **kwargs)
    return Body(body_id)

class Body(object):
    def __init__(self, bodyID): #info, **kwargs):
        #self.id = utils.load_model(info, **kwargs)
        self.id = bodyID
        self.base_link = -1
        self.static_mass = 0
        self.BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])
        self.DynamicsInfo = namedtuple('DynamicsInfo', ['mass', 'lateral_friction', 'local_inertia_diagonal', 
                                                        'local_inertial_pos', 'local_inertial_orn',
                                                        'restitution', 'rolling_friction', 'spinning_friction',
                                                        'contact_damping', 'contact_stiffness'])
        self.num_joints = p.getNumJoints(self.id, physicsClientId=CLIENT)
        self.joints = [Joint(self, j) for j in xrange(self.num_joints)]
        self.num_links = p.getNumJoints(self.id, physicsClientId=CLIENT)
        self.links = [Link(self, l) for l in xrange(self.num_links)]
        self.all_links = [Link(self, self.base_link)] + self.links
        # get_link_info = get_dynamics_info
        # joint id -> Joint Class is just self.joints[jointID]

    def get_info(self):
        return self.BodyInfo(*p.getBodyInfo(self.id, physicsClientId=CLIENT))

    def get_base_name(self):
        return self.get_info().base_name.decode(encoding='UTF-8')

    def get_body_name(self):
        return self.get_info().body_name.decode(encoding='UTF-8')

    def get_name(self):
        name = self.get_body_name()
        if name == '':
            name = 'body'
        return '{}{}'.format(name, int(self.id))

    def remove_body(self):
        if (CLIENT, self.id) in utils.INFO_FROM_BODY:
            del utils.INFO_FROM_BODY[CLIENT, self.id]
        return p.removeBody(self.id, physicsClientId=CLIENT)

    def set_color(self, color):
        p.changeVisualShape(self.id, -1, rgbaColor=color)

    def set_texture(self, textureFile):
        texture_id = p.loadTexture(textureFile)
        p.changeVisualShape(self.id, -1, textureUniqueId=texture_id)

    def get_pose(self):
        return p.getBasePositionAndOrientation(self.id, physicsClientId=CLIENT)

    def get_transform(self):
        return geometry.tform_from_pose(self.get_pose())

    def get_point(self):
        return self.get_pose()[0]

    def get_quat(self):
        return self.get_pose()[1] # [x,y,z,w]

    def get_euler(self):
        return geometry.euler_from_quat(self.get_quat())

    def get_base_values(self):
        return utils.base_values_from_pose(self.get_pose())

    def set_pose(self, pose):
        (point, quat) = pose
        p.resetBasePositionAndOrientation(self.id, point, quat, physicsClientId=CLIENT)

    def set_transform(self, transform):
        self.set_pose(geometry.pose_from_tform(transform))

    def set_point(self, point):
        self.set_pose((point, self.get_quat()))

    def set_quat(self, quat):
        self.set_pose((self.get_point(), quat))

    def set_euler(self, euler):
        self.set_quat(geometry.quat_from_euler(euler))

    def set_base_values(self, values):
        _, _, z = self.get_point()
        x, y, theta = values
        self.set_point((x, y, z))
        self.set_quat(geometry.z_rotation(theta))

    def get_velocity(self):
        linear, angular = p.getBaseVelocity(self.id, physicsClientId=CLIENT)
        return linear, angular # [x,y,z], [wx,wy,wz]

    def set_velocity(self, linear=None, angular=None):
        if linear is not None:
            p.resetBaseVelocity(self.id, linearVelocity=linear, physicsClientId=CLIENT)
        if angular is not None:
            p.resetBaseVelocity(self.id, angularVelocity=angular, physicsClientId=CLIENT)

    def is_rigid_body(self):
        for j in self.joints:
            if j.is_movable():
                return False
        return True

    def joint_from_name(self, name): 
        for j in self.joints:
            if j.get_joint_name() == name:
                return j
        raise ValueError

    def link_from_name(self, name):
        if name == self.get_base_name():
            #return self.base_link
            return Link(self, self.base_link)
        for link in self.links:
            if link.get_link_name() == name:
                return link
        raise ValueError(self, name)

    def has_joint(self, name):
        try:
            self.joint_from_name(name)
        except ValueError:
            return False
        return True

    def has_link(self, name):
        try:
            self.link_from_name(name)
        except ValueError:
            return False
        return True

    def joints_from_names(self, names):
        return tuple(self.joint_from_name(name) for name in names)

    def child_link_from_joint(self, joint): #XXX input type?
        return joint # link

    def parent_joint_from_link(self, link): #XXX input type?
        return link # joint

    def joint_from_movable(self, index):
        return self.joints[index]

    def get_configuration(self): 
        return self.get_joint_positions(self.get_movable_joints())

    def set_configuration(self, values): 
        self.set_joint_positions(self.get_movable_joints(), values)

    def get_full_configuration(self):
        # Cannot alter fixed joints
        return self.get_joint_positions(self.joints)

    def get_labeled_configuration(self): 
        movable_joints = self.get_movable_joints()
        return dict(zip(self.get_joint_names(movable_joints),
                        self.get_joint_positions(movable_joints)))

    def format_joint_input(self, joints):
        # Process through all of the input types and return actual joints
        if joints is None: 
            # If none, return all joints
            return self.joints
        elif isinstance(joints, int):
            # Return individual joint from id number
            return [self.joints[joints]]
        elif isinstance(joints, str):
            # Return individual joint from name
            return [self.joint_from_name(joints)]
        elif isinstance(joints, Joint):
            # Return individual joint from joint type
            return [joints]
        elif isinstance(joints, list):
            # Sort thru list and return based on type
            return [self.format_joint_input(j)[0] for j in joints] 
        else: 
            raise ValueError("Joint Input Type Not Supported")

    def get_movable_joints(self, joints=None): 
        return self.prune_fixed_joints(self.format_joint_input(joints))

    def prune_fixed_joints(self, joints=None):
        return [j for j in self.format_joint_input(joints) if j.is_movable()]

    def get_joint_names(self, joints=None):
        return [j.get_joint_name() for j in self.format_joint_input(joints)]

    def get_joint_names(self, joints=None):
        return [j.get_joint_name() for j in self.format_joint_input(joints)]

    def get_joint_positions(self, joints=None):
        return tuple(j.get_joint_position() for j in self.format_joint_input(joints))

    def get_joint_velocities(self, joints=None):
        return tuple(j.get_joint_velocity() for j in self.format_joint_input(joints))

    def wrap_positions(self, joints, positions): 
        assert len(joints) == len(positions)
        fjoints = self.format_joint_input(joints)
        return [j.wrap_position(position) for j, position in zip(fjoints, positions)]

    def violates_limits(self, joints, values):
        fjoints = self.format_joint_input(joints)
        return any(j.violates_limit(v) for j, v in zip(fjoints, values))

    def set_joint_positions(self, joints, values): 
        assert len(joints) == len(values)
        for joint, value in zip(self.format_joint_input(joints), values):
            joint.set_joint_position(value)

    def get_min_limits(self, joints=None):
        return [joint.get_min_limit() for joint in self.format_joint_input(joints)]

    def get_max_limits(self, joints=None):
        return [joint.get_max_limit() for joint in self.format_joint_input(joints)]

    def movable_from_joints(self, joints=None):
        fjoints = self.format_joint_input(joints)
        movable_from_original = {o: m for m, o in enumerate(self.get_movable_joints())}
        return [movable_from_original[joint.jointID] for joint in fjoints]

    def get_custom_limits(self, joints=None, custom_limits={}, circular_limits=utils.UNBOUNDED_LIMITS):
        joint_limits = []
        for joint in self.format_joint_input(joints):
            if joint in custom_limits:
                joint_limits.append(custom_limits[joint])
            elif joint.is_circular():
                joint_limits.append(circular_limits)
            else:
                joint_limits.append(joint.get_joint_limits())
        return zip(*joint_limits)

    def is_fixed_base(self):
        return self.get_mass() == self.static_mass 

    def get_num_links(self):
        return len(self.links)

    def get_adjacent_links(self):
        adjacent = set()
        for link in self.links: 
            parent = link.get_link_parent()
            adjacent.add((link, parent))
            #adjacent.add((parent, link))
        return adjacent

    def get_adjacent_fixed_links(self):
        return list(filter(lambda item: not (item[0].parentJoint).is_movable(),
                           self.get_adjacent_links()))

    def are_links_adjacent(self, link1, link2):
        return (link1.get_link_parent() == link2) or \
               (link2.get_link_parent() == link1)

    def get_all_link_parents(self):
        return {link: link.get_link_parent() for link in self.links}

    def get_all_link_children(self):
        children = {}
        for child, parent in self.get_all_link_parents().items():
            if parent not in children:
                children[parent] = []
            children[parent].append(child)
        return children

    def get_fixed_links(self):
        edges = defaultdict(list)
        for link, parent in self.get_adjacent_fixed_links():
            edges[link].append(parent)
            edges[parent].append(link)
        visited = set()
        fixed = set()
        for initial_link in self.links:
            if initial_link in visited:
                continue
            cluster = [initial_link]
            queue = deque([initial_link])
            visited.add(initial_link)
            while queue:
                for next_link in edges[queue.popleft()]:
                    if next_link not in visited:
                        cluster.append(next_link)
                        queue.append(next_link)
                        visited.add(next_link)
            fixed.update(itertools.product(cluster, cluster))
        return fixed

    def get_relative_pose(self, link1, link2):
        world_from_link1 = link1.get_link_pose()
        world_from_link2 = link2.get_link_pose()
        link2_from_link1 = geometry.multiply(geometry.invert(world_from_link2), world_from_link1)
        return link2_from_link1

    def get_dynamics_info(self, linkID=None):
        if linkID is None:
            linkID = self.base_link
        return self.DynamicsInfo(*p.getDynamicsInfo(self.id, linkID, physicsClientId=CLIENT))

    def get_mass(self, linkID=None):
        # TOOD: get full mass
        if linkID is None:
            linkID = self.base_link
        return self.get_dynamics_info(linkID).mass

    def set_dynamics(self, linkID=None, **kwargs):
        # TODO: iterate over all links
        if linkID is None:
            linkID = self.base_link
        p.changeDynamics(self.id, linkID, physicsClientId=CLIENT, **kwargs)

    def set_mass(self, mass, linkID=None):
        if linkID is None:
            linkID = self.base_link
        self.set_dynamics(link=linkID, mass=mass)

    def set_static(self):
        for link in self.all_links:
            self.set_mass(mass=self.static_mass, linkID=link.linkID)

    def dump_body(self):
        print('Body id: {} | Name: {} | Rigid: {} | Fixed: {}'.format(
            self.id, self.get_body_name(), self.is_rigid_body(), self.is_fixed_base()))
        for j in self.joints:
            if j.is_movable():
                print('Joint id: {} | Name: {} | Type: {} | Circular: {} | Limits: {}'.format(
                    j.jointID, j.get_joint_name(), JOINT_TYPES[j.get_joint_type()],
                    j.is_circular(), j.get_joint_limits()))
        link = -1
        print('Link id: {} | Name: {} | Mass: {} | Collision: {} | Visual: {}'.format(
            link, self.get_base_name(), self.get_mass(),
            len(utils.get_collision_data(self, self.base_link)), -1)) # len(get_visual_data(body, link))))

        for link in self.links:
            pjoint = link.parentJoint
            joint_name = JOINT_TYPES[pjoint.get_joint_type()] if pjoint.is_fixed() else pjoint.get_joint_name() 
            print('Link id: {} | Name: {} | Joint: {} | Parent: {} | Mass: {} | Collision: {} | Visual: {}'.format(
                link.linkID, link.get_link_name(), joint_name,
                (link.get_link_parent()).get_link_name(), self.get_mass(link.linkID),
                len(utils.get_collision_data(self, link.linkID)), -1)) # len(get_visual_data(body, link)))) #XXX move this function from utils

