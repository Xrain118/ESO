"""PreGME 论文公式级 Python 参考实现。"""

from .equations import GRAVITY_M_S2
from .equations import PaperParameters
from .equations import load_paper_default
from .equations import paper_default_sha256
from .equations import validate_parameters
from .equations import variable_gain
from .envelope import envelope
from .envelope import envelope_dot
from .envelope import preset_trajectory
from .envelope import preset_trajectory_b
from .envelope import preset_trajectory_ddot
from .envelope import preset_trajectory_dot

__all__ = [
    "GRAVITY_M_S2",
    "PaperParameters",
    "envelope",
    "envelope_dot",
    "load_paper_default",
    "paper_default_sha256",
    "preset_trajectory",
    "preset_trajectory_b",
    "preset_trajectory_ddot",
    "preset_trajectory_dot",
    "validate_parameters",
    "variable_gain",
]
