"""性能包络与论文式 (5) 预设误差轨迹。"""

from typing import Any, Tuple

import numpy as np


_C_LIMIT_THRESHOLD = np.float64(1.0e-6)


def _time_vector(t: Any) -> Tuple[np.ndarray, bool]:
    try:
        value = np.asarray(t, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("参数 t 必须是标量或一维数值数组") from exc
    scalar = value.ndim == 0
    if scalar:
        value = value.reshape(1)
    elif value.ndim != 1:
        raise ValueError("参数 t 必须是标量或形状为 (N,) 的一维数组")
    if not np.all(np.isfinite(value)):
        raise ValueError("参数 t 必须全部为有限数")
    if np.any(value < 0.0):
        raise ValueError("参数 t 必须大于等于 0")
    return value, scalar


def _axis_vector(value: Any, name: str) -> np.ndarray:
    try:
        converted = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {name} 必须是标量或三轴数值") from exc
    if converted.ndim == 0:
        converted = np.full(3, converted, dtype=np.float64)
    elif converted.shape != (3,):
        raise ValueError(f"参数 {name} 的形状必须为 (3,)")
    if not np.all(np.isfinite(converted)):
        raise ValueError(f"参数 {name} 必须全部为有限数")
    return converted


def _finish(value: np.ndarray, scalar_time: bool) -> np.ndarray:
    if scalar_time:
        return np.asarray(value[0], dtype=np.float64)
    return np.asarray(value, dtype=np.float64)


def _envelope_inputs(t: Any, rho0: Any, rho_inf: Any, l: Any):
    time, scalar_time = _time_vector(t)
    rho0_axis = _axis_vector(rho0, "rho0")
    rho_inf_axis = _axis_vector(rho_inf, "rho_inf")
    l_axis = _axis_vector(l, "l")
    if np.any(rho_inf_axis <= 0.0):
        raise ValueError("参数 rho_inf 必须逐轴大于 0")
    if np.any(rho0_axis <= rho_inf_axis):
        raise ValueError("参数 rho0 必须逐轴大于 rho_inf")
    if np.any(l_axis <= 0.0):
        raise ValueError("参数 l 必须逐轴大于 0")
    return time, scalar_time, rho0_axis, rho_inf_axis, l_axis


def envelope(t: Any, rho0: Any, rho_inf: Any, l: Any) -> np.ndarray:
    """计算性能包络定义式 ``rho(t)``。

    单位继承被约束状态：位置通道为 NED m，姿态通道为误差四元数向量部；
    ``l`` 为 s⁻¹、``t`` 为 s。沿用论文 SI 数值约定 A-01。
    """

    time, scalar_time, rho0_axis, rho_inf_axis, l_axis = _envelope_inputs(
        t, rho0, rho_inf, l
    )
    decay = np.exp(-time[:, None] * l_axis[None, :])
    value = (rho0_axis - rho_inf_axis) * decay + rho_inf_axis
    return _finish(value, scalar_time)


def envelope_dot(t: Any, rho0: Any, rho_inf: Any, l: Any) -> np.ndarray:
    """计算性能包络解析一阶导数 ``rho_dot(t)``。

    输出单位为对应状态单位/s，坐标继承位置 NED 或姿态误差轴；依赖 A-01。
    """

    time, scalar_time, rho0_axis, rho_inf_axis, l_axis = _envelope_inputs(
        t, rho0, rho_inf, l
    )
    decay = np.exp(-time[:, None] * l_axis[None, :])
    value = -l_axis * (rho0_axis - rho_inf_axis) * decay
    return _finish(value, scalar_time)


def preset_trajectory_b(beta0: Any, beta_dot0: Any, l: Any) -> np.ndarray:
    """由论文式 (5) 初值条件计算 ``b=l*beta0+beta_dot0``。

    ``beta0`` 必须由调用者设为初始误差，``beta_dot0`` 为初始误差导数；
    坐标和状态单位继承位置 NED 或姿态误差轴，``l`` 为 s⁻¹。依赖 A-01。
    """

    beta0_axis = _axis_vector(beta0, "beta0")
    beta_dot0_axis = _axis_vector(beta_dot0, "beta_dot0")
    l_axis = _axis_vector(l, "l")
    if np.any(l_axis <= 0.0):
        raise ValueError("参数 l 必须逐轴大于 0")
    return np.asarray(l_axis * beta0_axis + beta_dot0_axis, dtype=np.float64)


def _trajectory_inputs(t: Any, beta0: Any, beta_dot0: Any, l: Any, c: Any):
    time, scalar_time = _time_vector(t)
    beta0_axis = _axis_vector(beta0, "beta0")
    beta_dot0_axis = _axis_vector(beta_dot0, "beta_dot0")
    l_axis = _axis_vector(l, "l")
    c_axis = _axis_vector(c, "c")
    if np.any(l_axis <= 0.0):
        raise ValueError("参数 l 必须逐轴大于 0")
    if np.any(c_axis <= 0.0):
        raise ValueError("参数 c 必须逐轴大于 0")
    b_axis = l_axis * beta0_axis + beta_dot0_axis
    return time, scalar_time, beta0_axis, l_axis, c_axis, b_axis


def preset_trajectory(
    t: Any, beta0: Any, beta_dot0: Any, l: Any, c: Any
) -> np.ndarray:
    """计算论文式 (5) 的预设误差轨迹 ``beta(t)``。

    ``beta0=error(0)``、``beta_dot0=error_dot(0)`` 由调用者保证；位置通道
    沿 NED，姿态通道沿误差四元数向量部。``l,c`` 为 s⁻¹。依赖 A-01；
    对 ``0<c<1e-6`` 使用 A-03 的可去奇点极限。
    """

    time, scalar_time, beta0_axis, l_axis, c_axis, b_axis = _trajectory_inputs(
        t, beta0, beta_dot0, l, c
    )
    value = np.empty((time.size, 3), dtype=np.float64)
    for axis in range(3):
        decay_l = np.exp(-l_axis[axis] * time)
        if c_axis[axis] < _C_LIMIT_THRESHOLD:
            value[:, axis] = (beta0_axis[axis] + b_axis[axis] * time) * decay_l
        else:
            one_minus_decay_c = -np.expm1(-c_axis[axis] * time)
            value[:, axis] = (
                beta0_axis[axis] + b_axis[axis] * one_minus_decay_c / c_axis[axis]
            ) * decay_l
    return _finish(value, scalar_time)


def preset_trajectory_dot(
    t: Any, beta0: Any, beta_dot0: Any, l: Any, c: Any
) -> np.ndarray:
    """计算论文式 (5) 预设误差轨迹的解析一阶导数。

    输出单位为误差单位/s，坐标继承位置 NED 或姿态误差轴；初值前置条件
    同 ``preset_trajectory``。依赖 A-01，并采用 A-03 的小 ``c`` 极限。
    """

    time, scalar_time, beta0_axis, l_axis, c_axis, b_axis = _trajectory_inputs(
        t, beta0, beta_dot0, l, c
    )
    value = np.empty((time.size, 3), dtype=np.float64)
    for axis in range(3):
        decay_l = np.exp(-l_axis[axis] * time)
        if c_axis[axis] < _C_LIMIT_THRESHOLD:
            value[:, axis] = (
                b_axis[axis]
                - l_axis[axis] * (beta0_axis[axis] + b_axis[axis] * time)
            ) * decay_l
        else:
            decay_lc = np.exp(-(l_axis[axis] + c_axis[axis]) * time)
            one_minus_decay_c = -np.expm1(-c_axis[axis] * time)
            value[:, axis] = (
                -l_axis[axis] * beta0_axis[axis] * decay_l
                + b_axis[axis] * decay_lc
                - b_axis[axis]
                * l_axis[axis]
                * one_minus_decay_c
                * decay_l
                / c_axis[axis]
            )
    return _finish(value, scalar_time)
def preset_trajectory_ddot(
    t: Any, beta0: Any, beta_dot0: Any, l: Any, c: Any
) -> np.ndarray:
    """计算论文式 (5) 预设误差轨迹的解析二阶导数。

    输出单位为误差单位/s²，坐标继承位置 NED 或姿态误差轴；初值前置条件
    同 ``preset_trajectory``。依赖 A-01，并采用 A-03 的小 ``c`` 极限。
    """

    time, scalar_time, beta0_axis, l_axis, c_axis, b_axis = _trajectory_inputs(
        t, beta0, beta_dot0, l, c
    )
    value = np.empty((time.size, 3), dtype=np.float64)
    for axis in range(3):
        decay_l = np.exp(-l_axis[axis] * time)
        if c_axis[axis] < _C_LIMIT_THRESHOLD:
            value[:, axis] = (
                l_axis[axis] ** 2
                * (beta0_axis[axis] + b_axis[axis] * time)
                - 2.0 * l_axis[axis] * b_axis[axis]
            ) * decay_l
        else:
            decay_lc = np.exp(-(l_axis[axis] + c_axis[axis]) * time)
            one_minus_decay_c = -np.expm1(-c_axis[axis] * time)
            value[:, axis] = (
                l_axis[axis] ** 2 * beta0_axis[axis] * decay_l
                - b_axis[axis]
                * (2.0 * l_axis[axis] + c_axis[axis])
                * decay_lc
                + b_axis[axis]
                * l_axis[axis] ** 2
                * one_minus_decay_c
                * decay_l
                / c_axis[axis]
            )
    return _finish(value, scalar_time)
