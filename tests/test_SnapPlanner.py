#!/usr/bin/env python
import numpy, numpy.testing
import openravepy
import unittest
from prpy.planning.base import PlanningError
from prpy.planning.snap import SnapPlanner

class SnapPlannerTest(unittest.TestCase):
    # This configuration is in collision with the environment, but is not in
    # self-collision.
    config_env_collision = numpy.array([
        3.63026273e-01,  -1.54688036e+00,  -1.30000000e+00,
        2.34703418e+00,   3.28152338e-01,  -1.10662864e+00,
       -2.07807269e-01
    ])

    # This configuration is in self-collision, but is not in collision with the
    # environment.
    config_self_collision = numpy.array([
        2.60643264e+00,   1.97222205e+00,  -8.24298541e-16,
        2.79154009e+00,   1.30899694e+00,  -4.71027738e-16,
        0.00000000e+00
    ])

    # These waypoints have the following relationship:
    # - all three configurations are collision-free
    # - (config_segment1, config_segment2) in collision with the environment
    # - (config_segment2, config_segment3) is collision free.
    # - both segments are self-collision free
    config_segment1 = numpy.array([
        1.23760308e-01,   6.05769772e-01,  -5.00000000e-02,
        1.33403907e+00,   4.47724617e-01,  -3.14811779e-01,
       -1.90265540e+00
    ])
    config_segment2 = numpy.array([
        1.12376031e+00,   6.05769772e-01,  -5.00000000e-02,
        1.33403907e+00,   4.47724617e-01,  -3.14811779e-01,
       -1.90265540e+00
    ])
    config_segment3 = numpy.array([
        1.50376031e+00,   6.05769772e-01,  -5.00000000e-02,
        1.33403907e+00,   4.47724617e-01,  -3.14811779e-01,
       -1.90265540e+00
    ])

    def setUp(self):
        self.env = openravepy.Environment()
        self.env.Load('data/wamtest2.env.xml')
        self.robot = self.env.GetRobot('BarrettWAM')
        self.body = self.env.GetKinBody('mug-table')
        self.bodies = set(self.env.GetBodies())

        self.planner = SnapPlanner()

        with self.env:
            manipulator = self.robot.GetManipulator('arm')
            self.robot.SetActiveManipulator(manipulator)
            self.robot.SetActiveDOFs(manipulator.GetArmIndices())

            # The Segway is in contact with the floor, which can cause some
            # problems with planning.
            self.env.Remove(self.env.GetKinBody('floor'))

            # Compute a small motion that is larger than the collision checking
            # radius. This will be used to test endpoint conditions.
            dof_resolutions = self.robot.GetActiveDOFResolutions()
            self.config_epsilon = numpy.zeros(len(dof_resolutions))
            self.config_epsilon[0] += 0.5 * dof_resolutions[0]

    def test_PlanToConfiguration_LineIsValid_ReturnsTrajectory(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_segment3)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        self.robot.SetActiveDOFValues(self.config_segment2)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        # Test.
        traj = self.planner.PlanToConfiguration(self.robot, self.config_segment3)

        # Assert.
        self.assertEqual(traj.GetNumWaypoints(), 2)
        self.assertEqual(traj.GetConfigurationSpecification(),
                         self.robot.GetActiveConfigurationSpecification())
        numpy.testing.assert_allclose(traj.GetWaypoint(0), self.config_segment2)
        numpy.testing.assert_allclose(traj.GetWaypoint(1), self.config_segment3)

    def test_PlanToConfiguration_WithinResolution_ReturnsTrajectory(self):
        goal_config = self.config_segment1 + self.config_epsilon

        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_segment1)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        # Test.
        traj = self.planner.PlanToConfiguration(self.robot, goal_config)

        # Assert.
        self.assertEqual(traj.GetNumWaypoints(), 2)
        self.assertEqual(traj.GetConfigurationSpecification(),
                         self.robot.GetActiveConfigurationSpecification())
        numpy.testing.assert_allclose(traj.GetWaypoint(0), self.config_segment1)
        numpy.testing.assert_allclose(traj.GetWaypoint(1), goal_config)

    def test_PlanToConfiguration_StartInCollision_ThrowsPlanningError(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_env_collision)
        self.assertTrue(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        # Test and assert.
        goal_config = self.config_env_collision + self.config_epsilon

        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(self.robot, goal_config)

    def test_PlanToConfiguration_StartInSelfCollision_ThrowsPlanningError(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_self_collision)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertTrue(self.robot.CheckSelfCollision())

        # Test and assert.
        goal_config = self.config_self_collision + self.config_epsilon

        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(self.robot, goal_config)

    def test_PlanToConfiguration_SegmentInCollision_ThrowsPlanningError(self):
        # Setup and sanity checks.
        self.robot.SetActiveDOFValues(self.config_segment2)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        self.robot.SetActiveDOFValues(self.config_segment1)
        self.assertFalse(self.env.CheckCollision(self.robot))
        self.assertFalse(self.robot.CheckSelfCollision())

        # Test and assert.
        with self.assertRaises(PlanningError):
            self.planner.PlanToConfiguration(self.robot, self.config_segment2)

    def test_CloneRobot(self):
        self.robot.SetActiveDOFValues(self.config_self_collision)
        self.assertTrue(self.robot.CheckSelfCollision())

        cloned_robot = openravepy.RaveCreateRobot(self.env, self.robot.GetXMLId())
        cloned_robot.Clone(self.robot, 0)
        cloned_robot.SetName(self.robot.GetName() + '_clone')
        self.env.Add(cloned_robot)

        self.assertTrue(cloned_robot.CheckSelfCollision())

    def test_CloneBroken(self):
        self.robot.SetActiveDOFValues(self.config_self_collision)
        self.assertTrue(self.robot.CheckSelfCollision())

        cloned_env = openravepy.Environment()
        cloned_env.Clone(self.env, openravepy.CloningOptions.Bodies)

        cloned_robot = cloned_env.GetRobot(self.robot.GetName())
        self.assertTrue(cloned_robot.CheckSelfCollision())

    def test_CloneWorking(self):
        self.robot.SetActiveDOFValues(self.config_self_collision + 10 * self.config_epsilon)

        cloned_env = openravepy.Environment()
        cloned_env.Clone(self.env, openravepy.CloningOptions.Bodies)

        cloned_robot = cloned_env.GetRobot(self.robot.GetName())
        cloned_robot.SetActiveDOFValues(self.config_self_collision)
        self.assertTrue(cloned_robot.CheckSelfCollision())

if __name__ == '__main__':
    unittest.main()
