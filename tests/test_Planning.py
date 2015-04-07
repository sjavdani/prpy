#!/usr/bin/env python
import openravepy
import unittest
import numpy
import warnings
from openravepy import Environment
from prpy.planning.base import PlanningError
from prpy.planning.cbirrt import CBiRRTPlanner
from prpy.planning.ompl import OMPLPlanner, OMPLSimplifier
from prpy.planning.retimer import ParabolicRetimer
from prpy.planning.mac_smoother import MacSmoother
from numpy.testing import assert_allclose

from planning_tests.base import BasePlannerTest
from planning_tests.PlanToConfiguration import PlanToConfigurationTest
from planning_tests.PlanToEndEffectorPose import PlanToEndEffectorPoseTest
from planning_tests.RetimeTrajectory import RetimeTrajectoryTest
from planning_tests.ShortcutPath import ShortcutPathTest
from planning_tests.SmoothTrajectory import SmoothTrajectoryTest

#VerifyTrajectory = openravepy.planningutils.VerifyTrajectory


class CBiRRTPlannerTests(BasePlannerTest,
                         PlanToConfigurationTest,
                         PlanToEndEffectorPoseTest,
                         unittest.TestCase):
    planner_factory = CBiRRTPlanner


class OMPLPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       unittest.TestCase):
    planner_factory = OMPLPlanner


class OMPLSimplifierTests(BasePlannerTest,
                          ShortcutPathTest,
                          unittest.TestCase):
    from prpy.planning.ompl import OMPLSimplifier
    planner_factory = OMPLSimplifier

    def setUp(self):
        BasePlannerTest.setUp(self)
        ShortcutPathTest.setUp(self)


class ParabolicRetimerTests(BasePlannerTest,
                            RetimeTrajectoryTest,
                            unittest.TestCase):
    planner_factory = ParabolicRetimer

    def setUp(self):
        BasePlannerTest.setUp(self)
        RetimeTrajectoryTest.setUp(self)


if __name__ == '__main__':
    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Warn)

    unittest.main()
