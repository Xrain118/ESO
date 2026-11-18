"""论文式 (3) 变增益 ESO 与式 (7) 位置 ESO 的参考实现。"""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Tuple

import numpy as np

from .equations import PaperParameters
from .equations import variable_gain


def _readonly_vector(value: Any, field: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {field} 必须是三轴数值") from exc
    if vector.shape != (3,):
        raise ValueError(f"参数 {field} 的形状必须为 (3,)")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"参数 {field} 必须全部为有限数")
    vector = vector.copy()
    vector.setflags(write=False)
    return vector


def _runtime_vector(value: Any) -> Tuple[Optional[np.ndarray], Optional["EsoUpdateStatus"]]:
    try:
        vector = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return None, EsoUpdateStatus.INVALID_SHAPE
    if vector.shape != (3,):
        return None, EsoUpdateStatus.INVALID_SHAPE
    if not np.all(np.isfinite(vector)):
        return None, EsoUpdateStatus.NONFINITE_INPUT
    return vector.copy(), None


def _positive_float(value: Any, field: str, allow_zero: bool = False) -> np.float64:
    try:
        converted = np.float64(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {field} 必须是数值") from exc
    if not np.isfinite(converted):
        raise ValueError(f"参数 {field} 必须是有限数")
    if allow_zero:
        if converted < 0.0:
            raise ValueError(f"参数 {field} 必须大于等于 0")
    elif converted <= 0.0:
        raise ValueError(f"参数 {field} 必须大于 0")
    return converted


@dataclass(frozen=True)
class PositionEsoConfig:
    """位置 ESO 配置；NED 三轴增益及允许采样周期均使用 SI 单位。"""

    alpha: np.ndarray
    epsilon: np.ndarray
    w: np.float64
    d: np.float64
    minimum_period_s: np.float64
    maximum_period_s: np.float64

    def __post_init__(self) -> None:
        alpha = _readonly_vector(self.alpha, "alpha")
        epsilon = _readonly_vector(self.epsilon, "epsilon")
        if np.any(alpha <= 0.0):
            raise ValueError("参数 alpha 必须逐轴大于 0")
        if np.any((epsilon <= 0.0) | (epsilon >= 1.0)):
            raise ValueError("参数 epsilon 必须逐轴位于 (0,1)")
        w = _positive_float(self.w, "w")
        d = _positive_float(self.d, "d", allow_zero=True)
        minimum_period_s = _positive_float(
            self.minimum_period_s, "minimum_period_s"
        )
        maximum_period_s = _positive_float(
            self.maximum_period_s, "maximum_period_s"
        )
        if maximum_period_s <= minimum_period_s:
            raise ValueError("参数 maximum_period_s 必须大于 minimum_period_s")
        object.__setattr__(self, "alpha", alpha)
        object.__setattr__(self, "epsilon", epsilon)
        object.__setattr__(self, "w", w)
        object.__setattr__(self, "d", d)
        object.__setattr__(self, "minimum_period_s", minimum_period_s)
        object.__setattr__(self, "maximum_period_s", maximum_period_s)


def position_eso_config_from_paper(
    parameters: PaperParameters,
    minimum_period_s: float,
    maximum_period_s: float,
) -> PositionEsoConfig:
    """从论文参数和 M2 fixture 周期窗口构造式 (7) 的配置。

    位置通道使用 NED，``alpha`` 为 s⁻¹，``epsilon`` 无量纲，周期为 s；
    变增益沿用论文 SI 数值约定 A-01，不提供任何硬编码回退值。
    """

    if not isinstance(parameters, PaperParameters):
        raise ValueError("参数 parameters 必须是 PaperParameters")
    return PositionEsoConfig(
        alpha=parameters.position.eso.alpha,
        epsilon=parameters.position.eso.epsilon,
        w=parameters.variable_gain.w,
        d=parameters.variable_gain.d,
        minimum_period_s=minimum_period_s,
        maximum_period_s=maximum_period_s,
    )


def _validated_kernel_inputs(
    h: Any,
    y: Any,
    u: Any,
    alpha: Any,
    epsilon: Any,
    w: Any,
    d: Any,
):
    state = _readonly_vector(h, "h")
    measurement = _readonly_vector(y, "y")
    known_input = _readonly_vector(u, "u")
    alpha_vector = _readonly_vector(alpha, "alpha")
    epsilon_vector = _readonly_vector(epsilon, "epsilon")
    if np.any(alpha_vector <= 0.0):
        raise ValueError("参数 alpha 必须逐轴大于 0")
    if np.any((epsilon_vector <= 0.0) | (epsilon_vector >= 1.0)):
        raise ValueError("参数 epsilon 必须逐轴位于 (0,1)")
    w_value = _positive_float(w, "w")
    d_value = _positive_float(d, "d", allow_zero=True)
    return (
        state,
        measurement,
        known_input,
        alpha_vector,
        epsilon_vector,
        w_value,
        d_value,
    )


def variable_gain_eso_rk4_step(
    h: Any,
    held_y: Any,
    held_u: Any,
    alpha: Any,
    epsilon: Any,
    w: float,
    d: float,
    dt: float,
) -> np.ndarray:
    """以 RK4 推进论文式 (3) 的三轴变增益 ESO 状态。

    ``held_y`` 与 ``held_u`` 在整个区间内按前一帧零阶保持；位置实例的
    ``h/y`` 为 NED m/s、``u`` 为 NED m/s²、``dt`` 为 s。变增益复用式
    (4)，并沿用 A-01。函数只返回更新后的 ``h``，不修改任何入参。
    """

    (
        state,
        measurement,
        known_input,
        alpha_vector,
        epsilon_vector,
        w_value,
        d_value,
    ) = _validated_kernel_inputs(h, held_y, held_u, alpha, epsilon, w, d)
    dt_value = _positive_float(dt, "dt")

    def derivative(stage_state: np.ndarray) -> np.ndarray:
        innovation = measurement - stage_state
        return (
            alpha_vector * variable_gain(innovation, w_value, d_value) / epsilon_vector
            + known_input
        )

    k1 = derivative(state)
    k2 = derivative(state + 0.5 * dt_value * k1)
    k3 = derivative(state + 0.5 * dt_value * k2)
    k4 = derivative(state + dt_value * k3)
    next_state = state + dt_value * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    return _readonly_vector(next_state, "h_next")


def variable_gain_eso_estimate(
    h: Any,
    y: Any,
    alpha: Any,
    epsilon: Any,
    w: float,
    d: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """在给定状态上解析计算论文式 (3) 的创新和扰动估计。

    位置实例的 ``h/y`` 为 NED m/s，创新为 NED m/s，``delta_hat`` 为
    NED m/s²；使用更新后的状态和当前测量求值，不以数值差分反推。依赖
    论文式 (4) 和 A-01。
    """

    state = _readonly_vector(h, "h")
    measurement = _readonly_vector(y, "y")
    alpha_vector = _readonly_vector(alpha, "alpha")
    epsilon_vector = _readonly_vector(epsilon, "epsilon")
    if np.any(alpha_vector <= 0.0):
        raise ValueError("参数 alpha 必须逐轴大于 0")
    if np.any((epsilon_vector <= 0.0) | (epsilon_vector >= 1.0)):
        raise ValueError("参数 epsilon 必须逐轴位于 (0,1)")
    w_value = _positive_float(w, "w")
    d_value = _positive_float(d, "d", allow_zero=True)
    innovation = measurement - state
    disturbance_hat = (
        alpha_vector * variable_gain(innovation, w_value, d_value) / epsilon_vector
    )
    return (
        _readonly_vector(innovation, "innovation"),
        _readonly_vector(disturbance_hat, "disturbance_hat"),
    )


@dataclass(frozen=True)
class PositionEsoEstimate:
    """式 (7) 的 NED 位置 ESO 有效估计。"""

    h_v_ned_m_s: np.ndarray
    innovation_ned_m_s: np.ndarray
    disturbance_hat_ned_m_s2: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "h_v_ned_m_s", _readonly_vector(self.h_v_ned_m_s, "h_v_ned_m_s")
        )
        object.__setattr__(
            self,
            "innovation_ned_m_s",
            _readonly_vector(self.innovation_ned_m_s, "innovation_ned_m_s"),
        )
        object.__setattr__(
            self,
            "disturbance_hat_ned_m_s2",
            _readonly_vector(
                self.disturbance_hat_ned_m_s2, "disturbance_hat_ned_m_s2"
            ),
        )


class EsoUpdateStatus(str, Enum):
    """位置 ESO 单次更新状态，与冻结的时间戳/失效语义一一对应。"""

    INITIALIZED = "initialized"
    INTEGRATED = "integrated"
    DUPLICATE = "duplicate"
    INVALID_ZERO_TIMESTAMP = "invalid_zero_timestamp"
    TIMESTAMP_REGRESSION = "timestamp_regression"
    DT_OUT_OF_RANGE = "dt_out_of_range"
    NONFINITE_INPUT = "nonfinite_input"
    INVALID_SHAPE = "invalid_shape"
    NEEDS_RESET = "needs_reset"


@dataclass(frozen=True)
class EsoUpdate:
    """位置 ESO 更新结果；失效结果的 ``estimate`` 必须为 ``None``。"""

    status: EsoUpdateStatus
    estimate: Optional[PositionEsoEstimate]
    valid: bool
    integrated: bool


class PositionEso:
    """论文式 (7) 的三轴位置变增益 ESO 状态机。

    速度和观测器状态使用 NED m/s，已知输入及扰动估计使用 NED m/s²。
    区间内保持前一帧速度/输入，到达当前时间戳后再用当前速度计算创新。
    时间或数值故障会锁存，必须显式 ``reset()`` 后才可重新初始化。
    """

    def __init__(self, config: PositionEsoConfig) -> None:
        if not isinstance(config, PositionEsoConfig):
            raise ValueError("参数 config 必须是 PositionEsoConfig")
        self._config = config
        self.reset()

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def needs_reset(self) -> bool:
        return self._needs_reset

    def reset(self) -> None:
        """清空状态；下一条有效样本按 ``h_v(0)=v(0)`` 重新初始化。"""

        self._initialized = False
        self._needs_reset = False
        self._last_timestamp_us: Optional[int] = None
        self._h: Optional[np.ndarray] = None
        self._held_velocity: Optional[np.ndarray] = None
        self._held_input: Optional[np.ndarray] = None
        self._cached_estimate: Optional[PositionEsoEstimate] = None

    def _fault(self, status: EsoUpdateStatus) -> EsoUpdate:
        self._initialized = False
        self._needs_reset = True
        self._cached_estimate = None
        return EsoUpdate(status=status, estimate=None, valid=False, integrated=False)

    def update(
        self, velocity_ned: Any, input_ned: Any, timestamp_us: int
    ) -> EsoUpdate:
        """消费一个 PX4 微秒时间戳样本并更新位置 ESO。

        重复样本为良性且不消费载荷；零值、倒退、周期越界、形状错误和
        非有限输入均锁存失效。有效估计中的数组始终为只读 ``float64``。
        """

        if self._needs_reset:
            return EsoUpdate(
                status=EsoUpdateStatus.NEEDS_RESET,
                estimate=None,
                valid=False,
                integrated=False,
            )
        if isinstance(timestamp_us, (bool, np.bool_)) or not isinstance(
            timestamp_us, (int, np.integer)
        ):
            return self._fault(EsoUpdateStatus.INVALID_ZERO_TIMESTAMP)
        timestamp = int(timestamp_us)
        if timestamp <= 0:
            return self._fault(EsoUpdateStatus.INVALID_ZERO_TIMESTAMP)

        if self._last_timestamp_us is not None:
            if timestamp < self._last_timestamp_us:
                return self._fault(EsoUpdateStatus.TIMESTAMP_REGRESSION)
            if timestamp == self._last_timestamp_us:
                return EsoUpdate(
                    status=EsoUpdateStatus.DUPLICATE,
                    estimate=self._cached_estimate,
                    valid=True,
                    integrated=False,
                )
            dt = np.float64((timestamp - self._last_timestamp_us) / 1_000_000.0)
            if (
                dt < self._config.minimum_period_s
                or dt > self._config.maximum_period_s
            ):
                return self._fault(EsoUpdateStatus.DT_OUT_OF_RANGE)

        velocity, velocity_error = _runtime_vector(velocity_ned)
        known_input, input_error = _runtime_vector(input_ned)
        runtime_error = velocity_error or input_error
        if runtime_error is not None:
            return self._fault(runtime_error)
        assert velocity is not None
        assert known_input is not None

        if not self._initialized:
            zero = np.zeros(3, dtype=np.float64)
            estimate = PositionEsoEstimate(velocity, zero, zero)
            self._initialized = True
            self._last_timestamp_us = timestamp
            self._h = velocity.copy()
            self._held_velocity = velocity.copy()
            self._held_input = known_input.copy()
            self._cached_estimate = estimate
            return EsoUpdate(
                status=EsoUpdateStatus.INITIALIZED,
                estimate=estimate,
                valid=True,
                integrated=False,
            )

        assert self._h is not None
        assert self._held_velocity is not None
        assert self._held_input is not None
        assert self._last_timestamp_us is not None
        dt = np.float64((timestamp - self._last_timestamp_us) / 1_000_000.0)
        next_h = variable_gain_eso_rk4_step(
            self._h,
            self._held_velocity,
            self._held_input,
            self._config.alpha,
            self._config.epsilon,
            self._config.w,
            self._config.d,
            dt,
        )
        innovation, disturbance_hat = variable_gain_eso_estimate(
            next_h,
            velocity,
            self._config.alpha,
            self._config.epsilon,
            self._config.w,
            self._config.d,
        )
        estimate = PositionEsoEstimate(next_h, innovation, disturbance_hat)
        self._h = next_h.copy()
        self._held_velocity = velocity.copy()
        self._held_input = known_input.copy()
        self._last_timestamp_us = timestamp
        self._cached_estimate = estimate
        return EsoUpdate(
            status=EsoUpdateStatus.INTEGRATED,
            estimate=estimate,
            valid=True,
            integrated=True,
        )
