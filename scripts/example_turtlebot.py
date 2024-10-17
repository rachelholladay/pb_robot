#!/usr/bin/env python

#from __future__ import print_function

import os
import IPython
import pb_robot

if __name__ == '__main__':
    # Launch pybullet
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.disable_real_time()
    pb_robot.utils.set_default_camera()

    # Create robot object 
    robot = pb_robot.turtlebot.Turtlebot() 
 
    # Add floor object 
    curr_path = os.getcwd()
    models_path = os.path.join(os.path.dirname(curr_path), 'models')
    floor_file = os.path.join(models_path, 'short_floor.urdf')
    floor = pb_robot.body.createBody(floor_file)

    IPython.embed()
    
    # Close out Pybullet
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
