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


class TSR(object):
    def __init__(self, T0_w=None, Tw_e=None, Bw=None, link=None,
                 children=None):
        self.T0_w = T0_w if T0_w is not None else numpy.eye(4)
        self.Tw_e = Tw_e if Tw_e is not None else numpy.eye(4)
        self.Bw = Bw if Bw is not None else numpy.zeros((6, 2))
        self.link = self.link
        self.children = children if children is not None else []

        assert self.T0_w.shape == (4, 4)
        assert self.Tw_e.shape == (4, 4)
        assert self.Bw.shape == (6, 2)

    def distance(self, q):
        pass

    def sample(self, num_samples=1, T_pre=None, T_post=None):
        # TODO: What should this be called?
        B_norm = numpy.random.uniform(low=0., high=1., size=(num_samples, 6))
        B = B_norm * (self.Bw[:, 1] - self.Bw[:, 0]) + self.Bw[:, 0]

        # TODO: Vectorize this.
        Ts = []
        for b in B:
            xyzypr = [ b[0], b[1], b[2], b[5], b[4], b[3]]
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

    @staticmethod
    def _matrix_dot(A, B):
        # WTF?!
        # http://numpy-discussion.10968.n7.nabble.com/vectorized-multi-matrix-multiplication-tp21225p21227.html
        return numpy.einsum('nij,njk->nik', A, B)

    def to_dict(self):
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
        from json import dumps
        return dumps(self.to_dict())

    def to_yaml(self):
        from yaml import dumps
        return dumps(self.to_dict())


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
