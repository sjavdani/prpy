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


class TSR(object):
    def __init__(self, T0_w=None, Tw_e=None, Bw=None, link=None,
                 children=None):
        self.T0_w = T0_w if T0_w is not None else numpy.eye(4)
        self.Tw_e = Tw_e if Tw_e is not None else numpy.eye(4)
        self.Bw = Bw if Bw is not None else numpy.zeros((6, 2))
        self.link = self.link
        self.children = children if children is not None else []

    def distance(self, q):
        pass

    def sample(self, num_samples):
        pass

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
