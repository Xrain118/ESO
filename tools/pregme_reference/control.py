"""论文式 (8)、(9)、(15)–(21) 的滑模误差与控制律参考实现。"""

from dataclasses import dataclass
from typing import Any

import numpy as np

from .attitude import _readonly_inertia
from .equations import GRAVITY_M_S2
from .equations import PaperParameters
from .eso import _positive_float
from .eso import _readonly_vector


_ROTATION_TOLERANCE = np.float64(1.0e-10)
_DIRECTION_TOLERANCE = np.float64(1.0e-12)
_QUATERNION_SCALAR_MINIMUM = np.float64(1.0e-8)


def _readonly_matrix(value: Any, field: str) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {field} 必须是 3x3 数值矩阵") from exc
    if matrix.shape != (3, 3):
        raise ValueError(f"参数 {field} 的形状必须为 (3,3)")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"参数 {field} 必须全部为有限数")
    matrix = matrix.copy()
    matrix.setflags(write=False)
    return matrix


def _rotation_matrix(value: Any, field: str) -> np.ndarray:
    rotation = _readonly_matrix(value, field)
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3, dtype=np.float64),
        rtol=0.0,
        atol=_ROTATION_TOLERANCE,
    ):
        raise ValueError(f"参数 {field} 必须是正交旋转矩阵")
    determinant = np.linalg.det(rotation)
    if not np.isfinite(determinant) or abs(determinant - 1.0) > _ROTATION_TOLERANCE:
        raise ValueError(f"参数 {field} 的行列式必须为 +1")
    return rotation


def _positive_gain(value: Any, field: str) -> np.ndarray:
    gain = _readonly_vector(value, field)
    if np.any(gain <= 0.0):
        raise ValueError(f"参数 {field} 必须逐轴大于 0")
    return gain


def _finite_scalar(value: Any, field: str) -> np.float64:
    try:
        scalar = np.float64(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {field} 必须是数值") from exc
    if not np.isfinite(scalar):
        raise ValueError(f"参数 {field} 必须是有限数")
    return scalar


def skew_matrix(vector: Any) -> np.ndarray:
    """返回满足 ``[v]x a = v × a`` 的 FRD/NED 通用反对称矩阵。"""

    value = _readonly_vector(vector, "vector")
    matrix = np.array(
        [
            [0.0, -value[2], value[1]],
            [value[2], 0.0, -value[0]],
            [-value[1], value[0], 0.0],
        ],
        dtype=np.float64,
    )
    matrix.setflags(write=False)
    return matrix


@dataclass(frozen=True)
class PositionControlConfig:
    """论文式 (9) 的标称总质量和 NED 三轴对角增益。"""

    total_mass_kg: np.float64
    k: np.ndarray
    lambda_: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "total_mass_kg", _positive_float(self.total_mass_kg, "total_mass_kg")
        )
        object.__setattr__(self, "k", _positive_gain(self.k, "k"))
        object.__setattr__(
            self, "lambda_", _positive_gain(self.lambda_, "lambda_")
        )


def position_control_config_from_paper(
    parameters: PaperParameters,
) -> PositionControlConfig:
    """从 YAML 论文参数构造式 (9) 配置，不提供质量或增益兜底值。"""

    if not isinstance(parameters, PaperParameters):
        raise ValueError("参数 parameters 必须是 PaperParameters")
    return PositionControlConfig(
        total_mass_kg=(
            parameters.nominal_platform.quadcopter_mass_kg
            + parameters.nominal_platform.robotic_arm_mass_kg
        ),
        k=parameters.position.control.k_diagonal,
        lambda_=parameters.position.control.lambda_diagonal,
    )


@dataclass(frozen=True)
class PositionErrorTerms:
    """论文式 (8) 的 NED 位置误差、预设轨迹偏差和滑模向量。"""

    tracking_error_ned_m: np.ndarray
    z_ned_m: np.ndarray
    z_dot_ned_m_s: np.ndarray
    sliding_ned_m_s: np.ndarray

    def __post_init__(self) -> None:
        for field in (
            "tracking_error_ned_m",
            "z_ned_m",
            "z_dot_ned_m_s",
            "sliding_ned_m_s",
        ):
            object.__setattr__(self, field, _readonly_vector(getattr(self, field), field))


def position_error_terms(
    position_ned_m: Any,
    velocity_ned_m_s: Any,
    desired_position_ned_m: Any,
    desired_velocity_ned_m_s: Any,
    beta_ned_m: Any,
    beta_dot_ned_m_s: Any,
    lambda_: Any,
) -> PositionErrorTerms:
    """计算论文式 (8) 的 ``p_tilde,z_p,z_p_dot,s_p``。

    全部向量沿 NED；位置单位为 m、速度和滑模向量为 m/s，``lambda_``
    为 s⁻¹。``beta`` 及导数必须由式 (5) 解析函数提供，本函数不做微分。
    """

    position = _readonly_vector(position_ned_m, "position_ned_m")
    velocity = _readonly_vector(velocity_ned_m_s, "velocity_ned_m_s")
    desired_position = _readonly_vector(
        desired_position_ned_m, "desired_position_ned_m"
    )
    desired_velocity = _readonly_vector(
        desired_velocity_ned_m_s, "desired_velocity_ned_m_s"
    )
    beta = _readonly_vector(beta_ned_m, "beta_ned_m")
    beta_dot = _readonly_vector(beta_dot_ned_m_s, "beta_dot_ned_m_s")
    lambda_gain = _positive_gain(lambda_, "lambda_")
    tracking_error = position - desired_position
    z = tracking_error - beta
    z_dot = velocity - desired_velocity - beta_dot
    sliding = z_dot + lambda_gain * z
    return PositionErrorTerms(tracking_error, z, z_dot, sliding)


def position_control_force(
    config: PositionControlConfig,
    errors: PositionErrorTerms,
    desired_acceleration_ned_m_s2: Any,
    beta_ddot_ned_m_s2: Any,
    disturbance_hat_ned_m_s2: Any,
) -> np.ndarray:
    """计算论文编号式 (9) 的 NED 控制力向量 ``T_vec``。

    实现采用 ``m_B+m_R``、``p_d_ddot`` 和 ``+K_p s_p`` 的冻结勘误；
    输出单位 N。标准重力采用 A-02，不施加饱和或 PX4 归一化映射。
    """

    if not isinstance(config, PositionControlConfig):
        raise ValueError("参数 config 必须是 PositionControlConfig")
    if not isinstance(errors, PositionErrorTerms):
        raise ValueError("参数 errors 必须是 PositionErrorTerms")
    desired_acceleration = _readonly_vector(
        desired_acceleration_ned_m_s2, "desired_acceleration_ned_m_s2"
    )
    beta_ddot = _readonly_vector(beta_ddot_ned_m_s2, "beta_ddot_ned_m_s2")
    disturbance_hat = _readonly_vector(
        disturbance_hat_ned_m_s2, "disturbance_hat_ned_m_s2"
    )
    gravity_ned = np.array([0.0, 0.0, GRAVITY_M_S2], dtype=np.float64)
    bracket = (
        gravity_ned
        + disturbance_hat
        - desired_acceleration
        - beta_ddot
        + config.lambda_ * errors.z_dot_ned_m_s
        + config.k * errors.sliding_ned_m_s
    )
    return _readonly_vector(config.total_mass_kg * bracket, "force_ned_n")


def position_known_acceleration(force_ned_n: Any, total_mass_kg: Any) -> np.ndarray:
    """按论文式 (6) 由 ``T_vec`` 计算位置 ESO 已知输入 ``u_v``。"""

    force = _readonly_vector(force_ned_n, "force_ned_n")
    mass = _positive_float(total_mass_kg, "total_mass_kg")
    value = np.array([0.0, 0.0, GRAVITY_M_S2], dtype=np.float64) - force / mass
    return _readonly_vector(value, "known_acceleration_ned_m_s2")


@dataclass(frozen=True)
class DesiredAttitude:
    """论文式 (15) 的 FRD 到 NED 期望旋转及三个 NED 列基向量。"""

    rotation_ned_from_frd: np.ndarray
    b1_ned: np.ndarray
    b2_ned: np.ndarray
    b3_ned: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "rotation_ned_from_frd",
            _rotation_matrix(self.rotation_ned_from_frd, "rotation_ned_from_frd"),
        )
        for field in ("b1_ned", "b2_ned", "b3_ned"):
            object.__setattr__(self, field, _readonly_vector(getattr(self, field), field))


def desired_attitude(force_ned_n: Any, yaw_ned_rad: Any) -> DesiredAttitude:
    """按论文式 (15) 由 NED 控制力和偏航构造 FRD→NED 的 ``R_d``。

    ``b1,b2,b3`` 作为旋转矩阵的列；力方向退化或与偏航参考共线时拒绝，
    不静默选择替代轴。PR4 不在此生成 ``omega_d`` 或 ``omega_d_dot``。
    """

    force = _readonly_vector(force_ned_n, "force_ned_n")
    yaw = _finite_scalar(yaw_ned_rad, "yaw_ned_rad")
    force_norm = np.linalg.norm(force)
    if force_norm <= _DIRECTION_TOLERANCE:
        raise ValueError("参数 force_ned_n 的模必须大于 1e-12 N")
    b3 = force / force_norm
    yaw_reference = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float64)
    b2_raw = np.cross(b3, yaw_reference)
    b2_norm = np.linalg.norm(b2_raw)
    if b2_norm <= _DIRECTION_TOLERANCE:
        raise ValueError("式 (15) 退化：控制力方向与偏航参考共线")
    b2 = b2_raw / b2_norm
    b1 = np.cross(b2, b3)
    rotation = np.column_stack((b1, b2, b3))
    return DesiredAttitude(rotation, b1, b2, b3)


def _quaternion_error(rotation_error: np.ndarray):
    trace_argument = 1.0 + np.trace(rotation_error)
    if trace_argument < -_ROTATION_TOLERANCE:
        raise ValueError("姿态误差旋转矩阵的四元数迹公式无效")
    q0 = np.float64(0.5 * np.sqrt(max(0.0, float(trace_argument))))
    if q0 <= _QUATERNION_SCALAR_MINIMUM:
        raise ValueError("姿态误差接近 180 度，Q 矩阵不可可靠求解")
    antisymmetric = np.array(
        [
            rotation_error[2, 1] - rotation_error[1, 2],
            rotation_error[0, 2] - rotation_error[2, 0],
            rotation_error[1, 0] - rotation_error[0, 1],
        ],
        dtype=np.float64,
    )
    qv = antisymmetric / (4.0 * q0)
    return q0, _readonly_vector(qv, "qv")


@dataclass(frozen=True)
class AttitudeErrorTerms:
    """论文式 (16)–(20) 的姿态运动学、预设轨迹偏差和滑模量。"""

    rotation_error: np.ndarray
    rotation_error_dot: np.ndarray
    q0: np.float64
    qv: np.ndarray
    q0_dot: np.float64
    qv_dot: np.ndarray
    q_matrix: np.ndarray
    q_matrix_dot: np.ndarray
    omega_tilde: np.ndarray
    z: np.ndarray
    z_dot: np.ndarray
    sliding: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "rotation_error", _rotation_matrix(self.rotation_error, "rotation_error")
        )
        for field in ("rotation_error_dot", "q_matrix", "q_matrix_dot"):
            object.__setattr__(self, field, _readonly_matrix(getattr(self, field), field))
        object.__setattr__(self, "q0", _finite_scalar(self.q0, "q0"))
        object.__setattr__(self, "q0_dot", _finite_scalar(self.q0_dot, "q0_dot"))
        for field in ("qv", "qv_dot", "omega_tilde", "z", "z_dot", "sliding"):
            object.__setattr__(self, field, _readonly_vector(getattr(self, field), field))


def attitude_error_terms(
    rotation_ned_from_frd: Any,
    desired_rotation_ned_from_frd: Any,
    angular_velocity_frd_rad_s: Any,
    desired_angular_velocity_rad_s: Any,
    beta: Any,
    beta_dot_s_inv: Any,
    lambda_: Any,
) -> AttitudeErrorTerms:
    """计算论文式 (16)、(19) 的姿态误差、运动学和滑模向量。

    ``R,R_d`` 均为 FRD→NED；``omega`` 位于实际 FRD，``omega_d`` 位于
    期望机体系，均为 rad/s。``beta`` 是误差四元数向量部的预设轨迹。
    """

    rotation = _rotation_matrix(rotation_ned_from_frd, "rotation_ned_from_frd")
    desired_rotation = _rotation_matrix(
        desired_rotation_ned_from_frd, "desired_rotation_ned_from_frd"
    )
    omega = _readonly_vector(angular_velocity_frd_rad_s, "angular_velocity_frd_rad_s")
    omega_d = _readonly_vector(
        desired_angular_velocity_rad_s, "desired_angular_velocity_rad_s"
    )
    beta_value = _readonly_vector(beta, "beta")
    beta_dot = _readonly_vector(beta_dot_s_inv, "beta_dot_s_inv")
    lambda_gain = _positive_gain(lambda_, "lambda_")

    rotation_error = desired_rotation.T @ rotation
    rotation_error = _rotation_matrix(rotation_error, "rotation_error")
    rotation_error_dot = (
        -skew_matrix(omega_d) @ rotation_error
        + rotation_error @ skew_matrix(omega)
    )
    omega_tilde = omega - rotation_error @ omega_d
    q0, qv = _quaternion_error(rotation_error)
    q_matrix_value = q0 * np.eye(3, dtype=np.float64) + skew_matrix(qv)
    q0_dot = np.float64(-0.5 * qv @ omega_tilde)
    qv_dot = 0.5 * q_matrix_value @ omega_tilde
    q_matrix_dot = q0_dot * np.eye(3, dtype=np.float64) + skew_matrix(qv_dot)
    z = qv - beta_value
    z_dot = qv_dot - beta_dot
    sliding = z_dot + lambda_gain * z
    return AttitudeErrorTerms(
        rotation_error=rotation_error,
        rotation_error_dot=rotation_error_dot,
        q0=q0,
        qv=qv,
        q0_dot=q0_dot,
        qv_dot=qv_dot,
        q_matrix=q_matrix_value,
        q_matrix_dot=q_matrix_dot,
        omega_tilde=omega_tilde,
        z=z,
        z_dot=z_dot,
        sliding=sliding,
    )


@dataclass(frozen=True)
class AttitudeControlResult:
    """论文式 (18)、(21) 的非线性项和机体 FRD 力矩。"""

    nonlinear_term_frd_rad_s2: np.ndarray
    torque_frd_n_m: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "nonlinear_term_frd_rad_s2",
            _readonly_vector(
                self.nonlinear_term_frd_rad_s2,
                "nonlinear_term_frd_rad_s2",
            ),
        )
        object.__setattr__(
            self,
            "torque_frd_n_m",
            _readonly_vector(self.torque_frd_n_m, "torque_frd_n_m"),
        )


def attitude_control_torque(
    inertia_kg_m2: Any,
    angular_velocity_frd_rad_s: Any,
    errors: AttitudeErrorTerms,
    desired_angular_velocity_rad_s: Any,
    desired_angular_acceleration_rad_s2: Any,
    beta_ddot_s_inv2: Any,
    disturbance_hat_frd_rad_s2: Any,
    lambda_: Any,
    k: Any,
) -> AttitudeControlResult:
    """计算论文式 (18) 的 ``f_q`` 与式 (21) 的 FRD 控制力矩。

    ``omega_d,omega_d_dot`` 由调用者显式提供，PR4 不猜测差分滤波器。
    惯量单位 kg·m²，角量使用 rad/s 与 rad/s²，输出单位 N·m；不施加饱和。
    """

    inertia = _readonly_inertia(inertia_kg_m2)
    omega = _readonly_vector(angular_velocity_frd_rad_s, "angular_velocity_frd_rad_s")
    if not isinstance(errors, AttitudeErrorTerms):
        raise ValueError("参数 errors 必须是 AttitudeErrorTerms")
    omega_d = _readonly_vector(
        desired_angular_velocity_rad_s, "desired_angular_velocity_rad_s"
    )
    omega_d_dot = _readonly_vector(
        desired_angular_acceleration_rad_s2,
        "desired_angular_acceleration_rad_s2",
    )
    beta_ddot = _readonly_vector(beta_ddot_s_inv2, "beta_ddot_s_inv2")
    disturbance_hat = _readonly_vector(
        disturbance_hat_frd_rad_s2, "disturbance_hat_frd_rad_s2"
    )
    lambda_gain = _positive_gain(lambda_, "lambda_")
    k_gain = _positive_gain(k, "k")

    gyroscopic_acceleration = np.linalg.solve(
        inertia, np.cross(omega, inertia @ omega)
    )
    nonlinear_term = (
        0.5 * errors.q_matrix_dot @ errors.omega_tilde
        - 0.5 * errors.q_matrix @ gyroscopic_acceleration
        - 0.5 * errors.q_matrix @ (errors.rotation_error_dot @ omega_d)
        - 0.5 * errors.q_matrix @ (errors.rotation_error @ omega_d_dot)
    )
    bracket = (
        -nonlinear_term
        - 0.5 * errors.q_matrix @ disturbance_hat
        + beta_ddot
        - lambda_gain * errors.z_dot
        - k_gain * errors.sliding
    )
    try:
        solved = np.linalg.solve(errors.q_matrix, bracket)
    except np.linalg.LinAlgError as exc:
        raise ValueError("姿态误差 Q 矩阵不可逆") from exc
    torque = 2.0 * inertia @ solved
    return AttitudeControlResult(nonlinear_term, torque)
