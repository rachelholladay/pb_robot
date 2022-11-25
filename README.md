# pb_robot

This is a fork of [Caelan's ss-pybullet](https://github.com/caelan/ss-pybullet) with a lot of the same base functionality but a different UI. This repo is still in heavy development both in the sense of (1) reformatting the UI and (2) adding other functionality. These instructions are also skewed toward the use of the Franka Emika Panda, since thats what I'm primarily using. 

## Installation

The first thing is to install the primary python dependencies: 
```
$ pip install numpy pybullet recordclass networkx
```

Given that, we now setup a catkin workspace. While there are not known dependencies, these instructions were writen from Ubuntu 18.04 and ROS Melodic. For this installation we will assume ROS is already installed. To create a catkin workspace (named `my-workspace` below): 
```
$ mkdir my-workspace && cd my-workspace
$ catkin config --extend /opt/ros/melodic
$ catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Next we want to checkout code. 
```
$ cd src
$ git clone https://github.com/rachelholladay/pb_robot.git
$ cd pb_robot
$ git checkout python3
$ cd ..
```
This package uses TSRs to define grasp sets and therefore we clone an additional package: 
```
$ git clone https://github.com/rachelholladay/tsr.git
$ cd tsr
$ git checkout python3
$ cd ..
```

Given that we can build the workspace:
```
$ cd ..
$ catkin build
```

The last piece of the installation is to compile the IKFast library for the robot. For the Panda:
```
$ cd src/pb_robot/src/pb_robot/ikfast/franka_panda
$ python setup.py build
```

## Repo Structure

Any object in a pybullet environment is a "body". We define a set of function that can operate on a body in the body class, `src/pb_robot/body.py`. A body may be articulated and have joints and links. If so, when the body is created, we can define joint classes and link classes, each of which have functions that operate on the joint and link objects, defined in `src/pb_robot/joints.py` and `src/pb_robot/links.py` respectively. 

A robot is an articulated body. However, since there may be several specialized functions for a robot, we create a robot class that inherits from the body class. For the panda, this is defined in `src/pb_robot/panda.py`. Here there are robot specific definitions and functions on the robot arm and robot hand. 

Originally, most of the functions were defined in a file that I've renamed `og_util.py`. I'm working to break the functionality up into smaller, more specific files (i.e. `aabb.py` and `viz.py`). This is still very much in development and is why there is a significant amount of legacy code lying around. Hopefully for basic use cases, whats provided in `panda.py` is sufficient.

Also, most of the functionality is treating PyBullet as a kinematic simulation, not considering dynamics. I'm current developing control methods and hope to add them in to the main functionality soon. 

## Example Usage

Below is `scripts/example_panda.py` to show some basic functionality:

```
# Launch pybullet
utils.connect(use_gui=True)
utils.disable_real_time()
utils.set_default_camera()

# Create robot object 
robot = pb_robot.panda.Panda()

# Add floor object 
objects_path = pb_robot.helper.getDirectory()
floor_file = os.path.join(objects_path, 'furniture/short_floor.urdf')
floor = pb_robot.body.createBody(floor_file)

# Example function on body object
print floor.get_transform()

# Example functions over robot arm
q = robot.arm.GetJointValues()
pose = robot.arm.ComputeFK(q)
pose[2, 3] -= 0.1
pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(pose), length=0.5, width=10)
newq = robot.arm.ComputeIK(pose)
if newq is not None:
    raw_input("Move to desired pose?")
    robot.arm.SetJointValues(newq)

IPython.embed()

# Close out Pybullet
utils.wait_for_user()
utils.disconnect()             
```
