#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: Siddhartha Srinivasa <siddh@cs.cmu.edu>
# Authors: Michael Koval <mkoval@cs.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import logging
import numpy
import openravepy
import time
from base import BasePlanner, PlanningError, PlanningMethod

logger = logging.getLogger('planning')


class GreedyIKPlanner(BasePlanner):
    def __init__(self):
        super(GreedyIKPlanner, self).__init__()

    def __str__(self):
        return 'GreedyIKPlanner'

    @PlanningMethod
    def PlanToEndEffectorPose(self, robot, goal_pose, timelimit=5.0,
                              **kw_args):
        """
        Plan to an end effector pose by first creating a geodesic
        trajectory in SE(3) from the starting end-effector pose to the goal
        end-effector pose, and then attempting to follow it exactly
        using PlanWorkspacePath.

        @param robot
        @param goal_pose desired end-effector pose
        @return traj
        """

        with robot:
            # Create geodesic trajectory in SE(3)
            manip = robot.GetActiveManipulator()
            start_pose = manip.GetEndEffectorTransform()
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            spec = openravepy.IkParameterization.\
                GetConfigurationSpecificationFromType(
                        openravepy.IkParameterizationType.Transform6D,
                        'linear')
            traj.Init(spec)
            traj.Insert(traj.GetNumWaypoints(),
                        openravepy.poseFromMatrix(start_pose))
            traj.Insert(traj.GetNumWaypoints(),
                        openravepy.poseFromMatrix(goal_pose))
            openravepy.planningutils.RetimeAffineTrajectory(
                        traj,
                        maxvelocities=0.1*numpy.ones(7),
                        maxaccelerations=0.1*numpy.ones(7))

        return self.PlanWorkspacePath(robot, traj, timelimit)

    @PlanningMethod
    def PlanToEndEffectorOffset(self, robot, direction, distance,
                                max_distance=None, timelimit=5.0,
                                **kw_args):

        """
        Plan to a desired end-effector offset with move-hand-straight
        constraint. movement less than distance will return failure.
        The motion will not move further than max_distance.
        @param robot
        @param direction unit vector in the direction of motion
        @param distance minimum distance in meters
        @param max_distance maximum distance in meters
        @param timelimit timeout in seconds
        @return traj
        """

        if distance < 0:
            raise ValueError('Distance must be non-negative.')
        elif numpy.linalg.norm(direction) == 0:
            raise ValueError('Direction must be non-zero')
        elif max_distance is not None and max_distance < distance:
            raise ValueError('Max distance is less than minimum distance.')

        # Normalize the direction vector.
        direction = numpy.array(direction, dtype='float')
        direction /= numpy.linalg.norm(direction)

        with robot:
            manip = robot.GetActiveManipulator()
            start_pose = manip.GetEndEffectorTransform()
            traj = openravepy.RaveCreateTrajectory(self.env, '')
            spec = openravepy.IkParameterization.\
                GetConfigurationSpecificationFromType(
                        openravepy.IkParameterizationType.Transform6D,
                        'linear')
            traj.Init(spec)
            traj.Insert(traj.GetNumWaypoints(),
                        openravepy.poseFromMatrix(start_pose))
            min_pose = start_pose
            min_pose[0:3, 3] = min_pose[0:3, 3] + distance*direction
            traj.Insert(traj.GetNumWaypoints(),
                        openravepy.poseFromMatrix(min_pose))
            if max_distance is not None:
                max_pose = start_pose
                max_pose[0:3, 3] = max_pose[0:3, 3] + \
                    max_distance*direction
                traj.Insert(traj.GetNumWaypoints(),
                            openravepy.poseFromMatrix(max_pose))
            openravepy.planningutils.RetimeAffineTrajectory(
                        traj,
                        maxvelocities=0.1*numpy.ones(7),
                        maxaccelerations=0.1*numpy.ones(7))

        return self.PlanWorkspacePath(robot, traj,
                                      timelimit, minWaypointIndex=1)

    @PlanningMethod
    def PlanWorkspacePath(self, robot, traj, timelimit=5.0,
                          minWaypointIndex=None, **kw_args):
        """
        Plan a configuration space path given a workspace path.
        All timing information is ignored.
        @param robot
        @param traj workspace trajectory
                    represented as OpenRAVE AffineTrajectory
        @param minWaypointIndex minimum waypoint index to reach
        @param timelimit timeout in seconds
        @return qtraj configuration space path
        """

        with robot:
            manip = robot.GetActiveManipulator()
            qtraj = openravepy.RaveCreateTrajectory(self.env, '')
            qtraj.Init(manip.GetArmConfigurationSpecification())

            active_dof_indices = manip.GetArmIndices()
            # Initial guess for workspace path timing
            dt = traj.GetDuration()
            # Smallest CSpace step at which to give up
            MIN_STEP = min(robot.GetDOFResolutions())/100
            ikf = openravepy.IkFilterOptions.CheckEnvCollisions

            start_time = time.time()
            t = 0.0
            try:
                while t < traj.GetDuration():
                    # Check for a timeout.
                    current_time = time.time()
                    if (timelimit is not None and
                            current_time - start_time > timelimit):
                        raise PlanningError('Reached time limit.')

                    # Hypothesize new configuration as closest IK to current
                    qcurr = robot.GetDOFValues(active_dof_indices)
                    qnew = manip.FindIKSolution(
                                openravepy.matrixFromPose(traj.Sample(t)[0:7]),
                                ikf)
                    infeasibleStep = True
                    if qnew is not None:
                        # Found an IK
                        step = abs(qnew - qcurr)
                        if (max(step) < MIN_STEP) and qtraj:
                            raise PlanningError('Not making progress.')
                        infeasibleStep = any(step -
                                             robot.GetDOFResolutions() > 0.)
                    if infeasibleStep:
                        # backtrack and try half the step
                        t = t - dt
                        dt = dt/2.0
                    else:
                        robot.SetDOFValues(qnew, active_dof_indices)
                        qtraj.Insert(qtraj.GetNumWaypoints(), qnew)
                        dt = dt*2.0
                    t = t + dt
            except PlanningError as e:
                # Throw an error if we haven't reached the minimum waypoint
                if minWaypointIndex is None:
                    minWaypointIndex = traj.GetNumWaypoints()
                cspec = traj.GetConfigurationSpecification()
                wpts = [traj.GetWaypoint(i) for i in range(minWaypointIndex)]
                dts = [cspec.ExtractDeltaTime(wpt) for wpt in wpts]
                minTime = numpy.sum(dts)
                if t < minTime:
                    raise
                # Otherwise we'll gracefully terminate.
                else:
                    logger.warning('Terminated early at time %f < %f: %s',
                                   t, traj.GetDuration(), e.message)

        return qtraj