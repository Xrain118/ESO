"""PreGME 论文公式级 Python 参考实现。"""

from .attitude import AttitudeEso
from .attitude import AttitudeEsoConfig
from .attitude import AttitudeEsoEstimate
from .attitude import AttitudeEsoUpdate
from .attitude import attitude_eso_config_from_paper
from .attitude import rigid_body_known_angular_acceleration
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
from .eso import EsoUpdate
from .eso import EsoUpdateStatus
from .eso import PositionEso
from .eso import PositionEsoConfig
from .eso import PositionEsoEstimate
from .eso import position_eso_config_from_paper
from .eso import variable_gain_eso_estimate
from .eso import variable_gain_eso_rk4_step

__all__ = [
    "GRAVITY_M_S2",
    "PaperParameters",
    "AttitudeEso",
    "AttitudeEsoConfig",
    "AttitudeEsoEstimate",
    "AttitudeEsoUpdate",
    "EsoUpdate",
    "EsoUpdateStatus",
    "PositionEso",
    "PositionEsoConfig",
    "PositionEsoEstimate",
    "attitude_eso_config_from_paper",
    "envelope",
    "envelope_dot",
    "load_paper_default",
    "paper_default_sha256",
    "position_eso_config_from_paper",
    "preset_trajectory",
    "preset_trajectory_b",
    "preset_trajectory_ddot",
    "preset_trajectory_dot",
    "rigid_body_known_angular_acceleration",
    "validate_parameters",
    "variable_gain",
    "variable_gain_eso_estimate",
    "variable_gain_eso_rk4_step",
]
