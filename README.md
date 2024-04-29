# pbrspot

This is a fork of [Rachel Holladay's pb_robot](https://github.com/rachelholladay/pb_robot) with a lot of the same base functionality but specifically tailored to simulating and controlling a [Boston Dynamics Spot](https://bostondynamics.com/products/spot/) Robot. This repo is still in heavy development both in the sense of (1) reformatting the UI and (2) adding other functionality. It aims to provide a good base for simulating the Spot in pybullet that can be built on.

## Installation

```
pip install -e .
```

Note that this repo supports python 3.10+. We recommend python 3.10.14.

## Repo Structure

Any object in a pybullet environment is a "body". We define a set of function that can operate on a body in the body class, `pbrspot/body.py`. A body may be articulated and have joints and links. If so, when the body is created, we can define joint classes and link classes, each of which have functions that operate on the joint and link objects, defined in `pbrspot/joints.py` and `pbrspot/links.py` respectively. 

A robot is an articulated body. However, since there may be several specialized functions for a robot, we create a robot class that inherits from the body class. For the Spot, this is defined in `pbrspot/spot.py`. Here there are robot specific definitions and functions on the robot body and robot hand. 

Originally, most of the functions were defined in a file that's been renamed `og_util.py`. [Rachel Holladay](https://people.csail.mit.edu/rholladay/) is working to break the functionality up into smaller, more specific files (i.e. `aabb.py` and `viz.py`). This is still very much in development and is why there is a significant amount of legacy code lying around. Hopefully for basic use cases, whats provided in `spot.py` is sufficient, and adding additional functionality shouldn't be too hard!

Also, most of the functionality is treating PyBullet as a kinematic simulation, not considering dynamics (though turning on physics is a top priority for the near future). Rachel is current developing control methods and hope to add them in to the main functionality soon. 

## Example Usage

See `scripts/example_spot.py` for example usage functionality.