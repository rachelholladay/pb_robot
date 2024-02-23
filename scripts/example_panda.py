#!/usr/bin/env python

#from __future__ import print_function

import os
import IPython
import pbrspot

if __name__ == '__main__':
    # Launch pybullet
    pbrspot.utils.connect(use_gui=True)
    pbrspot.utils.disable_real_time()
    pbrspot.utils.set_default_camera()

    # Create robot object 
    robot = pbrspot.panda.Panda() 
 
    # Add floor object 
    curr_path = os.getcwd()
    models_path = os.path.join(os.path.dirname(curr_path), 'models')
    floor_file = os.path.join(models_path, 'short_floor.urdf')
    floor = pbrspot.body.createBody(floor_file)

    # Example function on body object
    print(floor.get_transform())

    # Example functions over robot arm
    q = robot.arm.GetJointValues()
    pose = robot.arm.ComputeFK(q)
    pose[2, 3] -= 0.1
    pbrspot.viz.draw_pose(pbrspot.geometry.pose_from_tform(pose), length=0.5, width=10)
    newq = robot.arm.ComputeIK(pose)
    if newq is not None:
        input("Move to desired pose?")
        robot.arm.SetJointValues(newq)

    IPython.embed()
    
    # Close out Pybullet
    pbrspot.utils.wait_for_user()
    pbrspot.utils.disconnect()
