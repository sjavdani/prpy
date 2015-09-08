#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Pras Velagapudi <pkv@cs.cmu.edu>
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
import time
import itertools
import numpy
import openravepy
from base import (BasePlanner, PlanningMethod, PlanningError,
                  UnsupportedPlanningError)

logger = logging.getLogger(__name__)


class TSRPlanner(BasePlanner):
    def __init__(self, delegate_planner=None):
        super(TSRPlanner, self).__init__()
        self.delegate_planner = delegate_planner

    def __str__(self):
        if self.delegate_planner is not None:
            return 'TSRPlanner({:s})'.format(str(self.delegate_planner))
        else:
            return 'TSRPlanner'

    # TODO: Temporarily disabled based on Pras' feedback.
    """
    @PlanningMethod
    def PlanToIK(self, robot, goal_pose, **kw_args):
        from ..tsr import TSR, TSRChain

        tsr = TSR(T0_w=goal_pose, Tw_e=numpy.eye(4), Bw=numpy.zeros((6,2)))
        tsr_chain = TSRChain(sample_goal=True, TSR=tsr)

        return self.PlanToTSR(robot, [tsr_chain], **kw_args)
    """

    @PlanningMethod
    def PlanToTSR(self, robot, tsrchains, tsr_timeout=2.0,
                  num_attempts=3, chunk_size=1, ranker=None,
                  max_deviation=2*numpy.pi, **kw_args):
        """
        Plan to a desired TSR set using a-priori goal sampling.  This planner
        samples a fixed number of goals from the specified TSRs up-front, then
        uses another planner's PlanToConfiguration (chunk_size = 1) or
        PlanToConfigurations (chunk_size > 1) to plan to the resulting affine
        transformations.

        This planner will return failure if the provided TSR chains require
        any constraint other than goal sampling.

        @param robot the robot whose active manipulator will be used
        @param tsrchains a list of TSR chains that define a goal set
        @param num_attempts the maximum number of planning attempts to make
        @param chunk_size the number of sampled goals to use per planning call
        @param tsr_timeout the maximum time to spend sampling goal TSR chains
        @param ranker an IK ranking function to use over the IK solutions
        @param max_deviation the maximum per-joint deviation from current pose
                             that can be considered a valid sample.
        @return traj a trajectory that satisfies the specified TSR chains
        """
        # Delegate to robot.planner by default.
        delegate_planner = self.delegate_planner or robot.planner

        # Plan using the active manipulator.
        with robot.GetEnv():
            manipulator = robot.GetActiveManipulator()

            # Distance from current configuration is default ranking.
            if ranker is None:
                from ..ik_ranking import NominalConfiguration
                ranker = NominalConfiguration(manipulator.GetArmDOFValues(),
                                              max_deviation=max_deviation)

        # Test for tsrchains that cannot be handled.
        for tsrchain in tsrchains:
            if tsrchain.sample_start or tsrchain.constrain:
                raise UnsupportedPlanningError(
                    'Cannot handle start or trajectory-wide TSR constraints.')
        tsrchains = [t for t in tsrchains if t.sample_goal]

        # Create an iterator that cycles through each TSR chain.
        tsr_cycler = itertools.cycle(tsrchains)

        # Create an iterator that cycles TSR chains until the timelimit.
        tsr_timelimit = time.time() + tsr_timeout
        tsr_sampler = itertools.takewhile(
            lambda v: time.time() < tsr_timelimit, tsr_cycler)

        # Sample a list of TSR poses and collate valid IK solutions.
        from openravepy import (IkFilterOptions,
                                IkParameterization,
                                IkParameterizationType)
        ik_solutions = []
        for tsrchain in tsr_sampler:
            ik_param = IkParameterization(
                tsrchain.sample(), IkParameterizationType.Transform6D)
            ik_solution = manipulator.FindIKSolutions(
                ik_param, IkFilterOptions.CheckEnvCollisions,
                ikreturn=False, releasegil=True
            )
            if ik_solution.shape[0] > 0:
                ik_solutions.append(ik_solution)

        if len(ik_solutions) == 0:
            raise PlanningError('No collision-free IK solutions at goal TSRs.')

        # Sort the IK solutions in ascending order by the costs returned by the
        # ranker. Lower cost solutions are better and infinite cost solutions
        # are assumed to be infeasible.
        ik_solutions = numpy.vstack(ik_solutions)
        scores = ranker(robot, ik_solutions)
        valid_idxs = ~numpy.isposinf(scores)
        valid_scores = scores[valid_idxs]
        valid_solutions = ik_solutions[valid_idxs, :]
        ranked_indices = numpy.argsort(valid_scores)
        ranked_ik_solutions = valid_solutions[ranked_indices, :]

        # Group the IK solutions into sets of the specified size
        # (plan for each set of IK solutions together).
        ranked_ik_solution_sets = [
            ranked_ik_solutions[i:i+chunk_size, :]
            for i in range(0, ranked_ik_solutions.shape[0], chunk_size)
        ]

        num_attempts = min(len(ranked_ik_solution_sets), num_attempts)
        ik_set_list = enumerate(ranked_ik_solution_sets[:num_attempts])

        # Configure the robot to use the active manipulator for planning.
        p = openravepy.KinBody.SaveParameters
        with robot.CreateRobotStateSaver(p.ActiveDOF):
            robot.SetActiveDOFs(manipulator.GetArmIndices())

            # Try planning to each solution set in descending cost order.
            for i, ik_set in ik_set_list:
                try:
                    if ik_set.shape[0] > 1:
                        traj = delegate_planner.PlanToConfigurations(
                            robot, ik_set, **kw_args)
                    else:
                        traj = delegate_planner.PlanToConfiguration(
                            robot, ik_set[0], **kw_args)

                    logger.info('Planned to IK solution set %d of %d.',
                                i + 1, num_attempts)
                    return traj
                except PlanningError as e:
                    logger.warning(
                        'Planning to IK solution set %d of %d failed: %s',
                        i + 1, num_attempts, e)

        # If none of the planning attempts succeeded, report failure.
        raise PlanningError(
            'Planning to the top {:d} of {:d} IK solution sets failed.'
            .format(num_attempts, len(ranked_ik_solution_sets)))
