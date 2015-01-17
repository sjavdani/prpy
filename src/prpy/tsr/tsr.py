# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
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
# -*- coding: utf-8 -*-
import numpy
import kin

XYZYPR_TO_XYZRPY = [0, 1, 2, 5, 4, 3]

class TSR(object):
    def __init__(self, T0_w=None, Tw_e=None, Bw=None,
                 T0_link=None, Te_link=None, children=None):
        """ Defines a Task Space Region in SE(3)

        A TSR represents a contiguous region of poses of a child frame in SE(3)
        parameterized by bounded variations in translation and rotation of a
        task frame.  The task frame Tw is specified by the transformation T0_w
        from a parent frame to the task frame.  Within the task frame, pose is
        allowed to vary within the bounds of Bw, which specifies the min (first
        column) and max (second column) translation and rotation in (x, y, z,
        roll, pitch, yaw).  The child frame is defined by a fixed
        transformation Tw_e to from the varied task frame.

        Optionally, the parent and child frames can be associated with OpenRAVE
        links.  This is useful when the TSR constrains the pose of a
        manipulator or object.

        TSRs constraints can be composed hierarchically by associating the
        parent frame of one TSR to the child frame of another.  This can be
        specified by passing a list of child TSRs to this constructor or,
        later, appending to self.children. In this, the parent's Te_link must
        match the T0_link of all children (if specified).  This relationship is
        inferred when possible.

        @param T0_w transform from parent to task frame
        @param Tw_e transform from task to child frame
        @param Bw 6x2 constraint relaxation bounds
        @param T0_link optional OpenRAVE link for parent frame
        @param Te_link optional OpenRAVE link for child frame
        @param children collection of child TSR objects
        """
        self.T0_w = T0_w if T0_w is not None else numpy.eye(4)
        self.Tw_e = Tw_e if Tw_e is not None else numpy.eye(4)
        self.Bw = Bw if Bw is not None else numpy.zeros((6, 2))
        self.T0_link = link
        self.Te_link = Te_link
        self.parent_link = parent_link
        self.children = children if children is not None else []

        assert self.T0_w.shape == (4, 4)
        assert self.Tw_e.shape == (4, 4)
        assert self.Bw.shape == (6, 2)

        # Collates all children's T0_link's. These must all be the same since
        # they refer to our Te_link.
        Te_link_candidates = {
            child.T0_link for child in self.children if not None
        }
        if len(Te_link_candidates) > 1:
            raise ValueError(
                "Children have multiple values of T0_link: " +\
                str([ link.GetName() for link in Te_link_candidates ])
            )

        # Populate our Te_link from the children's T0_link if necessary.
        if len(Te_link_candidates) == 1:
            if self.Te_link is None:
                self.Te_link = Te_link_candidates[0]
            elif self.Te_link != Te_link_candidates[0]:
                raise ValueError(
                    "Child T0_link '{:s}' does not match parent's Te_link"
                    " '{:s}'.".format(child.T0_link.GetName(), Te_link.GetName())
                )

        # Downward-propagate our Te_link (which may have been inferred from a
        # child TSR) to all children TSRs. At this point we know we'll either
        # be: (1) setting the value to itself or (2) overwriting None.
        for child in self.children:
            child.T0_link = self.Te_link

    def distance(self, Te, T0=None, weight=None):
        """ Computes weighted Euclidean distance from a pose to this TSR.

        This method computes the weighted Euclidean distance from a target pose
        to the nearest pose in the TSR. The distance corresponds to the
        weighted Euclidean norm of this residual vector in (x, y, z, roll,
        pitch, yaw) specified in the task frame.

        Note that this is a purely geometric computation that does not consider
        kinematic feasibility. It may not be possible for self.child_link to
        reach the nearest pose in the TSR due to kinematic constraints. Solving
        the constrained problem requires global non-linear optimization and is,
        in the general case, intractible.

        @param Te target pose in the parent frame
        @param weight optional 6x1 Euclidean distance weight vector
        @param T0 optional transform from world to the parent frame
        @return weighted distance from this TSR to Te
        """
        if T0 is not None:
            Tw = numpy.dot(T0, self.T0_w)
        else:
            Tw = self.T0_w

        if weight is None:
            weight = numpy.ones(6)

        # Compute the transform from Tw to Tw_target. This is the residual in
        # the task frame, which is constrained by Bw.
        Tw_target = numpy.dot(Te, numpy.linalg.inv(Tw_e))
        Tw_relative = numpy.dot(numpy.linalg.inv(Tw), Tw_target)

        # Compute distance from the Bw AABB.
        xyzypr = kin.pose_to_xyzypr(Tw_relative)
        xyzrpy = xyzypr[XYZYPR_TO_XYZRPY]

        distance_vector = numpy.maximum(
            numpy.maximum(xyzrpy - self.Bw[:, 1], numpy.zeros(6)),
            numpy.maximum(self.Bw[:, 1] - xyzrpy, numpy.zeros(6))
        )
        return numpy.linalg.norm(weight * distance_vector, ord=2)

    def sample(self, num_samples=1, T_pre=None, T_post=None):
        """ Uniformly sample child frame poses from this TSR.

        This method uniformly samples num_samples transformations from the
        parent frame to the child frame of this TSR. This is implemented by
        uniformly sampling a relative transformation in the range of (x, y, z,
        roll, pitch, yaw) values defined by the Bw matrix. This transform is
        then pre-multiplied by T0_w and post-multiplied by Tw_e. If specified,
        the result is additionally pre- and post-multiplied by the optional
        parameters T_pre and T_post.

        @param num_samples optional number of samples
        @param T_pre optional transformation to be pre-multiplied to T0_w
        @param T_post optional transformation to be post-multiplied to Tw_e
        @return list of num_samples samples of T0_e
        """
        B_norm = numpy.random.uniform(low=0., high=1., size=(num_samples, 6))
        B = B_norm * (self.Bw[:, 1] - self.Bw[:, 0]) + self.Bw[:, 0]

        # TODO: Vectorize this.
        Ts = []
        for b in B:
            xyzypr = b[XYZYPR_TO_XYZRPY]
            Tw = kin.pose_to_H(kin.pose_from_xyzypr(xyzypr))
            T = numpy.dot(numpy.dot(T0_w, Tw), Tw_e)

            if T_pre:
                T = numpy.dot(T_pre, T)
            if T_post:
                T = numpy.dot(T, T_post)

            Ts.append(T)

        return Ts

    def sample_links(self, num_samples=1, T_pre=None):
        Ts_parent = self.sample(num_samples=num_samples, T_pre=T_pre)
        Ts_parent = numpy.array(Ts)
        D = {}

        if self.link:
            D[self.link] = Ts_parent

        for child in self.children:
            D_child = child.sample_links(num_samples=num_samples)

            # Any duplicate link transform is a zero measure set and cannot be
            # sampled. This is invalid.
            for link, Ts_child in D.iteritems():
                if link in D_child:
                    raise ValueError('Duplicate link transform for "{:s}" not'
                                     ' supported.'.format(self.link.GetName()))

                D[link] = self._matrix_dot(Ts_parent, Ts_child)

        return D

    def validate(self):
        """ Verifies the consistency of parent and child links in this tree.

        This method verifies that the invariant child.T0_link = self.Te_link
        for all children in this subtree. If the constraint is violated, this
        method raises a ValueError. Unlike the TSR constructor, this method
        does not attempt to infer the correct value for these properties if
        they are not set.
        """
        for child in self.children:
            if child.T0_link != self.Te_link:
                raise ValueError(
                    "Child T0_link '{:s}' does not match parent's Te_link"
                    " '{:s}'.".format(child.T0_link.GetName(), Te_link.GetName())
                )

            child.validate()

    def to_dict(self):
        """ Serializes this TSR, and all children, to a dict

        @return dictionary representation of this TSR
        """
        return {
            'T0_w': T_0w.tolist(),
            'Tw_e': Tw_e.tolist(),
            'Bw': Bw.tolist(),
            'link': {
                'body': link.GetParent().GetName(),
                'link': link.GetName(),
            },
            'children': [ child.to_dict() for child in self.children ],
        }

    def to_json(self):
        """ Serializes this TSR, and all children, to JSON

        @return string encoding this TSR as JSON
        """
        from json import dumps
        return dumps(self.to_dict())

    def to_yaml(self):
        """ Serializes this TSR, and all children, to YAML

        @return string encoding this TSR as YAML
        """
        from yaml import dumps
        return dumps(self.to_dict())

    @staticmethod
    def _matrix_dot(A, B):
        # WTF?!
        # http://numpy-discussion.10968.n7.nabble.com/vectorized-multi-matrix-multiplication-tp21225p21227.html
        return numpy.einsum('nij,njk->nik', A, B)

def TSRChain(tsrs):
    if not len(tsrs) == 0:
        raise ValueError('At least one TSR must be specified.')

    root_tsr = tsrs[0]
    if root_tsr.children:
        raise ValueError('Root TSR cannot have children.')

    prev_tsr = root_tsr
    for curr_tsr in tsrs[1:]:
        if curr_tsr.children:
            raise ValueError('TSR cannot have children.')

        prev_tsr.children = [curr_tsr]
        prev_tsr = curr_tsr

    return root_tsr
