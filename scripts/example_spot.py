#!/usr/bin/env python

#from __future__ import print_function

import os
import numpy
import scipy
import IPython
import pb_robot

def dumb_ik(arm, ik_target, start_q=None, max_iterations=5):
    '''
    @param ik_target the 4x4 transform we want to achieve
    @param seed_q the starting guess for the ik solver. If none is given we 
                use the current arm configuration
    @return Joint configuration q such that q = IK(ik_target) or
                None if no solution could be found
    '''
    if start_q is None:
        start_q = arm.GetJointValues()

    (lower_real, upper_real) = arm.GetJointLimits()
    eps = numpy.ones((7))*0.001
    lower = numpy.add(lower_real, eps)
    upper = numpy.subtract(upper_real, eps)

    def ik_cost(current_q, desired_pose):
        # Define the cost as geodesic distance between FK(current_pose) and desired pose
        current_pose = arm.ComputeFK(current_q)
        geodesic = pb_robot.geometry.GeodesicDistance(current_pose, desired_pose, r=1) #0.17)
        current_quat = pb_robot.geometry.quat_from_matrix(current_pose[0:3, 0:3])
        desired_quat = pb_robot.geometry.quat_from_matrix(desired_pose[0:3, 0:3])
        ang_dist = 0.5*pb_robot.geometry.quat_angle_between(current_quat, desired_quat)
        return max(geodesic, ang_dist)

    for _ in range(max_iterations):
        # Keep seeding it with the previous solution till solution is food enough
        (start_q, final_value, _, _, _) = scipy.optimize.fmin_slsqp(ik_cost, x0=start_q, args=(ik_target,), bounds=list(zip(lower, upper)), full_output=True, disp=False)

        # Result of optimization is sufficiently close that we return the solution
        if final_value < 1e-3: 
            return start_q
    return None

if __name__ == '__main__':
    # Launch pybullet
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.disable_real_time()
    pb_robot.utils.set_default_camera()

    # Create robot object 
    #robot = pb_robot.spot.Spot()
    #robot.set_point([0, 0, 0.69])

    robot = pb_robot.spot.SpotArm()
    robot.set_point([0, 0, 0.1])
 
    # Add floor object 
    curr_path = os.getcwd()
    models_path = os.path.join(os.path.dirname(curr_path), 'models')
    floor_file = os.path.join(models_path, 'short_floor.urdf')
    floor = pb_robot.body.createBody(floor_file)


    '''
    # Example functions over robot arm
    q = robot.arm.GetJointValues()
    pose = robot.arm.ComputeFK(q)
    pose[2, 3] -= 0.1
    pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(pose), length=0.5, width=10)
    newq = robot.arm.ComputeIK(pose)
    if newq is not None:
        input("Move to desired pose?")
        robot.arm.SetJointValues(newq)
    '''
    IPython.embed()
    
    # Close out Pybullet
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
