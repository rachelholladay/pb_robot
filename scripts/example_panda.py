#!/usr/bin/env python

#from __future__ import print_function

import os
import IPython
import pb_robot_spot

if __name__ == '__main__':
    # Launch pybullet
    pb_robot_spot.utils.connect(use_gui=True)
    pb_robot_spot.utils.disable_real_time()
    pb_robot_spot.utils.set_default_camera()

    # Create robot object 
    robot = pb_robot_spot.panda.Panda() 
 
    # Add floor object 
    curr_path = os.getcwd()
    models_path = os.path.join(os.path.dirname(curr_path), 'models')
    floor_file = os.path.join(models_path, 'short_floor.urdf')
    floor = pb_robot_spot.body.createBody(floor_file)

    # Example function on body object
    print(floor.get_transform())

    # Example functions over robot arm
    q = robot.arm.GetJointValues()
    pose = robot.arm.ComputeFK(q)
    pose[2, 3] -= 0.1
    pb_robot_spot.viz.draw_pose(pb_robot_spot.geometry.pose_from_tform(pose), length=0.5, width=10)
    newq = robot.arm.ComputeIK(pose)
    if newq is not None:
        input("Move to desired pose?")
        robot.arm.SetJointValues(newq)

    IPython.embed()
    
    # Close out Pybullet
    pb_robot_spot.utils.wait_for_user()
    pb_robot_spot.utils.disconnect()
