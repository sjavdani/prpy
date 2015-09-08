#!/usr/bin/env python

# Add this directory to PYTHONPATH when running inside nosetest.
import os.path, sys
sys.path = [os.path.abspath(os.path.dirname(__file__))] + sys.path

import unittest
from planning_tests.base import BasePlannerTest
from planning_tests.PlanToConfiguration import PlanToConfigurationTest
from planning_tests.PlanToEndEffectorPose import PlanToEndEffectorPoseTest
from planning_tests.RetimeTrajectory import RetimeTrajectoryTest
from planning_tests.ShortcutPath import ShortcutPathTest
from planning_tests.SmoothTrajectory import SmoothTrajectoryTest


class CBiRRTPlannerTests(BasePlannerTest,
                         PlanToConfigurationTest,
                         PlanToEndEffectorPoseTest,
                         unittest.TestCase):
    from prpy.planning.cbirrt import CBiRRTPlanner

    planner_factory = CBiRRTPlanner


class OMPLPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       unittest.TestCase):
    from prpy.planning.ompl import OMPLPlanner
    
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
    from prpy.planning.retimer import ParabolicRetimer

    planner_factory = ParabolicRetimer

    def setUp(self):
        BasePlannerTest.setUp(self)
        RetimeTrajectoryTest.setUp(self)


class BiRRTPlannerTests(BasePlannerTest,
                        PlanToConfigurationTest,
                        unittest.TestCase):
    from prpy.planning.openrave import BiRRTPlanner

    planner_factory = BiRRTPlanner

    def setUp(self):
        BasePlannerTest.setUp(self)


class GreedyIKPlannerTests(BasePlannerTest,
                           #PlanToEndEffectorPoseTest, # NOTE: Temporarily disabled.
                           unittest.TestCase):
    from prpy.planning.workspace import GreedyIKPlanner

    planner_factory = GreedyIKPlanner

    def setUp(self):
        BasePlannerTest.setUp(self)


class VectorFieldPlannerTests(BasePlannerTest,
                              #PlanToEndEffectorPoseTest, # NOTE: Temporarily disabled.
                              unittest.TestCase):
    from prpy.planning.vectorfield import VectorFieldPlanner

    planner_factory = VectorFieldPlanner

    def setUp(self):
        BasePlannerTest.setUp(self)


class SnapPlannerTests(BasePlannerTest,
                       PlanToConfigurationTest,
                       PlanToEndEffectorPoseTest,
                       unittest.TestCase):
    from prpy.planning.snap import SnapPlanner

    planner_factory = SnapPlanner

    def setUp(self):
        BasePlannerTest.setUp(self)

    # SnapPlanner only succeeds when the path is collision free. We guarantee
    # that this is the case by disabling everything in the environment.
    def test_PlanToConfiguration_GoalIsFeasible_FindsSolution(self):
        with self.env:
            self._disable_everything()
            super(SnapPlannerTests,
                self).test_PlanToConfiguration_GoalIsFeasible_FindsSolution()

    def test_PlanToEndEffectorPose_GoalIsFeasible_FindsSolution(self):
        with self.env:
            self._disable_everything()
            super(SnapPlannerTests,
                self).test_PlanToEndEffectorPose_GoalIsFeasible_FindsSolution()

    def _disable_everything(self):
        for body in self.env.GetBodies():
            body.Enable(False)



if __name__ == '__main__':
    import openravepy

    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()
    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

    unittest.main()
