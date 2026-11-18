"""PreGME 论文公式级 Python 参考实现。"""

from .attitude import AttitudeEso
from .attitude import AttitudeEsoConfig
from .attitude import AttitudeEsoEstimate
from .attitude import AttitudeEsoUpdate
from .attitude import attitude_eso_config_from_paper
from .attitude import rigid_body_known_angular_acceleration
from .control import AttitudeControlResult
from .control import AttitudeErrorTerms
from .control import DesiredAttitude
from .control import PositionControlConfig
from .control import PositionErrorTerms
from .control import attitude_control_torque
from .control import attitude_error_terms
from .control import desired_attitude
from .control import position_control_config_from_paper
from .control import position_control_force
from .control import position_error_terms
from .control import position_known_acceleration
from .control import skew_matrix
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
    "AttitudeControlResult",
    "AttitudeErrorTerms",
    "DesiredAttitude",
    "EsoUpdate",
    "EsoUpdateStatus",
    "PositionEso",
    "PositionEsoConfig",
    "PositionEsoEstimate",
    "PositionControlConfig",
    "PositionErrorTerms",
    "attitude_control_torque",
    "attitude_error_terms",
    "attitude_eso_config_from_paper",
    "envelope",
    "envelope_dot",
    "desired_attitude",
    "load_paper_default",
    "paper_default_sha256",
    "position_eso_config_from_paper",
    "position_control_config_from_paper",
    "position_control_force",
    "position_error_terms",
    "position_known_acceleration",
    "preset_trajectory",
    "preset_trajectory_b",
    "preset_trajectory_ddot",
    "preset_trajectory_dot",
    "rigid_body_known_angular_acceleration",
    "skew_matrix",
    "validate_parameters",
    "variable_gain",
    "variable_gain_eso_estimate",
    "variable_gain_eso_rk4_step",
]
