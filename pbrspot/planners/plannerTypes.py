#/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Goal and Constraint Types
'''

from enum import Enum

class GoalType(Enum):
    JOINT = 1
    TSR_EE = 2
    TSR_TOOL = 3

class ConstraintType(Enum):
    GOAL_JOINT = 1
    GOAL_EE = 2
    PATH_JOINT = 3
    PATH_EE = 4
