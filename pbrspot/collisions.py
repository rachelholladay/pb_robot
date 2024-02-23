from collections import namedtuple
from itertools import product, combinations
import numpy as np
import pybullet as p
import pbrspot

MAX_DISTANCE = 0
CLIENT = 0
BASE_LINK = -1

ContactResult = namedtuple('ContactResult', ['contactFlag', 'bodyUniqueIdA', 'bodyUniqueIdB',
                                             'linkIndexA', 'linkIndexB', 'positionOnA', 'positionOnB',
                                             'contactNormalOnB', 'contactDistance', 'normalForce'])


def get_collision_fn(body, joints, obstacles, attachments, self_collisions, custom_limits={}, **kwargs):
    check_link_pairs = get_self_link_pairs(body, joints) if self_collisions else []
    moving_links = frozenset(get_moving_links(body, joints))
    moving_bodies = [(body, moving_links)] + attachments
    check_body_pairs = list(product(moving_bodies, obstacles))  
    lower_limits, upper_limits = body.get_custom_limits(joints, custom_limits)

    def collision_fn(q):
        if not pbrspot.helper.all_between(lower_limits, q, upper_limits):
            return True
        body.set_joint_positions(joints, q) 
        for link1, link2 in check_link_pairs:
            if pairwise_link_collision(body, link1, body, link2):
                return True
        for body1, body2 in check_body_pairs:
            if pairwise_collision(body1, body2, **kwargs): 
                return True
        return False
    return collision_fn


def get_self_link_pairs(body, joints, disabled_collisions=set(), only_moving=True):
    moving_links = get_moving_links(body, joints)
    #fixed_links = list(set(body.links) - set(moving_links))
    moving_links_ids = [l.linkID for l in moving_links]
    fixed_links = [link for link in body.links if link.linkID not in moving_links_ids] 
    check_link_pairs = list(product(moving_links, fixed_links))
    if only_moving:
        check_link_pairs.extend(get_moving_pairs(body, joints))
    else:
        check_link_pairs.extend(combinations(moving_links, 2))
    check_link_pairs = list(filter(lambda pair: not body.are_links_adjacent(*pair), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                   (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs

def get_moving_links(body, joints):
    moving_links = set()
    for joint in joints:
        jlink = body.child_link_from_joint(joint)
        link = pbrspot.link.Link(body, jlink.jointID)
        if link not in moving_links:
            moving_links.update(link.get_link_subtree())
    return list(moving_links)

def get_moving_pairs(body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = get_moving_links(body, moving_joints)
    for link1, link2 in combinations(moving_links, 2):
        ancestors1 = set(link1.get_joint_ancestors()) & set(moving_joints)
        ancestors2 = set(link2.get_joint_ancestors()) & set(moving_joints)
        if ancestors1 != ancestors2:
            yield link1, link2

def pairwise_link_collision(body1, link1, body2, link2=BASE_LINK, max_distance=MAX_DISTANCE): # 10000
    return len(p.getClosestPoints(bodyA=body1.id, bodyB=body2.id, distance=max_distance,
                                  linkIndexA=link1.linkID, linkIndexB=link2.linkID,
                                  physicsClientId=CLIENT)) != 0 # getContactPoints


def body_collision(body1, body2, max_distance=MAX_DISTANCE): # 10000
    return len(p.getClosestPoints(bodyA=body1.id, bodyB=body2.id, distance=max_distance,
                                  physicsClientId=CLIENT)) != 0 # getContactPoints`

def pairwise_collision(body1, body2, **kwargs):
    if isinstance(body1, tuple) or isinstance(body2, tuple):
        body1, links1 = expand_links(body1)
        body2, links2 = expand_links(body2)
        return any_link_pair_collision(body1, links1, body2, links2, **kwargs)
    return body_collision(body1, body2, **kwargs)

def single_collision(body1, **kwargs):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2, **kwargs):
            return True
    return False

def any_link_pair_collision(body1, links1, body2, links2=None, **kwargs):
    # TODO: this likely isn't needed anymore
    if links1 is None:
        links1 = body1.all_links
    if links2 is None:
        links2 = body2.all_links
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return True
    return False

def link_pairs_collision(body1, links1, body2, links2=None, **kwargs):
    if links2 is None:
        links2 = body2.all_links
    for link1, link2 in product(links1, links2):
        if (body1 == body2) and (link1 == link2):
            continue
        if pairwise_link_collision(body1, link1, body2, link2, **kwargs):
            return True
    return False

def expand_links(body):
    body, links = body if isinstance(body, tuple) else (body, None)
    if links is None:
        links = body.all_links
    return body, links
