"""论文式 (13)、式 (14) 姿态变增益 ESO 的参考实现。"""

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from .equations import PaperParameters
from .eso import EsoUpdateStatus
from .eso import PositionEso
from .eso import PositionEsoConfig
from .eso import _readonly_vector


def _readonly_inertia(value: Any, field: str = "inertia_kg_m2") -> np.ndarray:
    try:
        inertia = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {field} 必须是 3x3 数值矩阵") from exc
    if inertia.shape != (3, 3):
        raise ValueError(f"参数 {field} 的形状必须为 (3,3)")
    if not np.all(np.isfinite(inertia)):
        raise ValueError(f"参数 {field} 必须全部为有限数")
    if not np.allclose(inertia, inertia.T, rtol=0.0, atol=1.0e-12):
        raise ValueError(f"参数 {field} 必须对称")
    try:
        np.linalg.cholesky(inertia)
    except np.linalg.LinAlgError as exc:
        raise ValueError(f"参数 {field} 必须正定") from exc
    inertia = inertia.copy()
    inertia.setflags(write=False)
    return inertia


def rigid_body_known_angular_acceleration(
    angular_velocity_frd_rad_s: Any,
    torque_frd_n_m: Any,
    inertia_kg_m2: Any,
) -> np.ndarray:
    """计算论文式 (13) 中不含扰动的已知角加速度 ``u_omega``。

    输入角速度、力矩和惯量均在机体 FRD 坐标系，单位分别为 rad/s、N·m
    和 kg·m²；输出为 FRD rad/s²。实现求解
    ``I u_omega = tau - omega × (I omega)``，不显式求逆。惯量必须有限、
    对称且正定；A-01 只影响后续式 (4)，本函数不修改任何入参。
    """

    angular_velocity = _readonly_vector(
        angular_velocity_frd_rad_s, "angular_velocity_frd_rad_s"
    )
    torque = _readonly_vector(torque_frd_n_m, "torque_frd_n_m")
    inertia = _readonly_inertia(inertia_kg_m2)
    right_hand_side = torque - np.cross(
        angular_velocity, inertia @ angular_velocity
    )
    acceleration = np.linalg.solve(inertia, right_hand_side)
    return _readonly_vector(acceleration, "known_angular_acceleration_frd_rad_s2")


@dataclass(frozen=True)
class AttitudeEsoConfig:
    """式 (14) 配置；三轴按机体 FRD 排列，周期使用秒。"""

    alpha: np.ndarray
    epsilon: np.ndarray
    w: np.float64
    d: np.float64
    minimum_period_s: np.float64
    maximum_period_s: np.float64

    def __post_init__(self) -> None:
        validated = PositionEsoConfig(
            alpha=self.alpha,
            epsilon=self.epsilon,
            w=self.w,
            d=self.d,
            minimum_period_s=self.minimum_period_s,
            maximum_period_s=self.maximum_period_s,
        )
        for field in (
            "alpha",
            "epsilon",
            "w",
            "d",
            "minimum_period_s",
            "maximum_period_s",
        ):
            object.__setattr__(self, field, getattr(validated, field))

    def _shared_config(self) -> PositionEsoConfig:
        return PositionEsoConfig(
            alpha=self.alpha,
            epsilon=self.epsilon,
            w=self.w,
            d=self.d,
            minimum_period_s=self.minimum_period_s,
            maximum_period_s=self.maximum_period_s,
        )


def attitude_eso_config_from_paper(
    parameters: PaperParameters,
    minimum_period_s: float,
    maximum_period_s: float,
) -> AttitudeEsoConfig:
    """从论文参数和 M2 fixture 周期窗口构造式 (14) 配置。

    姿态通道沿机体 FRD，``alpha`` 为 s⁻¹、``epsilon_omega`` 无量纲；
    变增益沿用论文 SI 数值约定 A-01，不提供硬编码回退值。
    """

    if not isinstance(parameters, PaperParameters):
        raise ValueError("参数 parameters 必须是 PaperParameters")
    return AttitudeEsoConfig(
        alpha=parameters.attitude.eso.alpha,
        epsilon=parameters.attitude.eso.epsilon,
        w=parameters.variable_gain.w,
        d=parameters.variable_gain.d,
        minimum_period_s=minimum_period_s,
        maximum_period_s=maximum_period_s,
    )


@dataclass(frozen=True)
class AttitudeEsoEstimate:
    """式 (14) 的机体 FRD 姿态 ESO 有效估计。"""

    h_omega_frd_rad_s: np.ndarray
    innovation_frd_rad_s: np.ndarray
    disturbance_hat_frd_rad_s2: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "h_omega_frd_rad_s",
            _readonly_vector(self.h_omega_frd_rad_s, "h_omega_frd_rad_s"),
        )
        object.__setattr__(
            self,
            "innovation_frd_rad_s",
            _readonly_vector(self.innovation_frd_rad_s, "innovation_frd_rad_s"),
        )
        object.__setattr__(
            self,
            "disturbance_hat_frd_rad_s2",
            _readonly_vector(
                self.disturbance_hat_frd_rad_s2,
                "disturbance_hat_frd_rad_s2",
            ),
        )


@dataclass(frozen=True)
class AttitudeEsoUpdate:
    """姿态 ESO 更新结果；状态枚举与共享 ESO 状态机一致。"""

    status: EsoUpdateStatus
    estimate: Optional[AttitudeEsoEstimate]
    valid: bool
    integrated: bool


class AttitudeEso:
    """论文式 (14) 的三轴姿态变增益 ESO 状态机。

    角速度和观测器状态使用机体 FRD rad/s，已知输入及扰动估计使用
    FRD rad/s²。数值推进、前一帧零阶保持、时间戳和锁存复位语义直接复用
    PR2 的式 (3) 唯一内核；首样本严格执行 ``h_omega(0)=omega(0)``。
    """

    def __init__(self, config: AttitudeEsoConfig) -> None:
        if not isinstance(config, AttitudeEsoConfig):
            raise ValueError("参数 config 必须是 AttitudeEsoConfig")
        self._observer = PositionEso(config._shared_config())

    @property
    def initialized(self) -> bool:
        return self._observer.initialized

    @property
    def needs_reset(self) -> bool:
        return self._observer.needs_reset

    def reset(self) -> None:
        """清空共享状态；下一有效样本按 ``h_omega=omega`` 初始化。"""

        self._observer.reset()

    def update(
        self,
        angular_velocity_frd: Any,
        known_angular_acceleration_frd: Any,
        timestamp_us: int,
    ) -> AttitudeEsoUpdate:
        """消费一个微秒时间戳的 FRD 角速度/已知角加速度样本。

        重复样本不积分且不消费载荷；时间、形状或非有限故障会锁存，必须
        显式 ``reset()``。有效返回数组均为只读 ``float64``。
        """

        shared = self._observer.update(
            angular_velocity_frd,
            known_angular_acceleration_frd,
            timestamp_us,
        )
        if shared.estimate is None:
            estimate = None
        else:
            estimate = AttitudeEsoEstimate(
                h_omega_frd_rad_s=shared.estimate.h_v_ned_m_s,
                innovation_frd_rad_s=shared.estimate.innovation_ned_m_s,
                disturbance_hat_frd_rad_s2=(
                    shared.estimate.disturbance_hat_ned_m_s2
                ),
            )
        return AttitudeEsoUpdate(
            status=shared.status,
            estimate=estimate,
            valid=shared.valid,
            integrated=shared.integrated,
        )
