__author__ = 'mhgou'
__version__ = '1.2.11'

from .graspnet import GraspNet
from .grasp import Grasp, GraspGroup, RectGrasp, RectGraspGroup

__all__ = [
    'GraspNet',
    'GraspNetEval',
    'Grasp',
    'GraspGroup',
    'RectGrasp',
    'RectGraspGroup',
]


def __getattr__(name):
    if name == 'GraspNetEval':
        from .graspnet_eval import GraspNetEval
        return GraspNetEval
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
