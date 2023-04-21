#!/usr/bin/env python

#from __future__ import print_function

import os
import numpy
import IPython
import pb_robot

if __name__ == '__main__':
    # Launch pybullet
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.disable_real_time()
    pb_robot.utils.set_default_camera()

    # Create robot object 
    robot = pb_robot.spot.Spot()
    robot.set_point([0, 0, 0])

    # Optionally, you could create just an arm, with no base. 
    #robot = pb_robot.spot.SpotArm()
    #robot.set_point([0, 0, 0])

    # Add floor object 
    curr_path = os.getcwd()
    models_path = os.path.join(os.path.dirname(curr_path), 'models')
    floor_file = os.path.join(models_path, 'short_floor.urdf')
    floor = pb_robot.body.createBody(floor_file)
    floor.set_point([0, 0, 0])

    # This is intended to show off functionality and provide examples
    # I suppose its somewhat in the style of a Jupyter notebook.. 
    # but without that nice incremental sections! So it'll just run everything!  

    ####################
    # FK and IK Examples
    ####################

    # Get the configuration of the arm
    q = robot.arm.GetJointValues()

    # Compute the forward kinematics
    pose = robot.arm.ComputeFK(q)

    # Note that you could also get the end effector (EE) pose directly 
    # (here pose == other_pose)
    other_pose = robot.arm.GetEETransform()

    # Slighly modify to generate a new pose
    pose[2, 3] += 0.1
    # Visualize this new pose
    pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(pose), length=0.5, width=10)
    # Compute IK for this new pose
    newq = robot.arm.ComputeIK(pose)
    # Not that ComputeIK() returns None if the IK solver couldn't find a solution. 
    # So we'd always want to check if the solution was found before setting the configuration
    if newq is not None:
        robot.arm.SetJointValues(newq)

    # Note that you can also ask for the IK solution that is closest to some seed configuration
    newq_seed = robot.arm.ComputeIK(pose, seed_q=q)

    # By default the IK solver is using the current base pose. 
    # But we can compute the IK for a specific base pose. 
    # Here lets create a base pose that moved slightly over
    movebase_pose = numpy.eye(4)
    movebase_pose[1, 3] += 0.05
    # Then we can solve IK for this base pose
    movebase_q = robot.arm.ComputeIK(pose, robot_worldF=movebase_pose)
    if movebase_q is not None:
        # I focused on the arm and not locomotion so we move the robot base
        # by teleporting
        robot.set_transform(movebase_pose)
        robot.arm.SetJointValues(movebase_q)

    # We can also control the hand, which is just a revolute joint. 
    # We can open the hand, which sets it to one joint limit
    robot.arm.hand.Open()
    # We can close it, which sets it to the other joint limit
    robot.arm.hand.Close()
    # Or we can specify the value of the joint
    robot.arm.hand.MoveTo(-0.25)

    # Teleport robot back to origin
    robot.set_point([0, 0, 0])


    #############################
    # Collision Checking Examples
    #############################

    # Lets modify the pose such that its in the floor, bwhaha!
    pose[2, 3] -= 0.4
    # Again, compute IK. Note that IK does not collision check
    floorq = robot.arm.ComputeIK(pose)
    # By default, the collision checker checks against all objects in the environment. 
    # We can also collision checks against a particular set of obstacles. In this example
    # I can do something silly and collision check against a empty set of obstacles. 
    # Hence withFloor is in collision but withoutFloor isn't.
    if floorq is not None:
        withFloor = robot.arm.isCollisionFree(floorq)
        withoutFloor = robot.arm.isCollisionFree(floorq, obstacles=[])
        print("Check against floor? {}. Check without floor? {}".format(withFloor, withoutFloor))


    ##########################
    # Motion Planning Examples 
    ##########################

    # Get a random configuration. By default, the configuration is collision
    # checked against all objects in the environment (similar to IsCollisionFree(), 
    # you can optionally pass in the set of obstacles to collision check against). 
    qrand = robot.arm.randomConfiguration()

    # Plan a collision-free joint space path from q to qrand
    # Note that this is my python, non-optimized implementation, so its not the 
    # fastest planners. Better engineers write fast planners, so blame 
    # the speed on me, not sample-based planning in general :) 
    path = robot.arm.birrt.PlanToConfiguration(robot.arm, q, qrand)
    # The path is a list of joint configurations (or None, if no path is found)
    # ExecutePositionPath just visualizes the path by teleporting between the 
    # positions - its a very dumb function. 
    # I do have most of the code to run path execution with pybullet physics, 
    # I'd just have to hook it up better
    if path is not None:
        robot.arm.ExecutePositionPath(path)

    IPython.embed()
    
    # Close out Pybullet
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
