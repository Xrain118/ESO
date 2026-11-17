"""PreGME 公共方程、论文参数加载与校验。"""

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Any, Mapping, Union

import numpy as np
import yaml


PathLike = Union[str, Path]
GRAVITY_M_S2 = 9.80665
_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_PAPER_PATH = Path("config/paper_default.yaml")


@dataclass(frozen=True)
class PaperIdentity:
    """论文版本和冻结状态。"""

    identifier: str
    revision: str
    source_sha256: str
    formulas_frozen: bool
    params_complete: bool


@dataclass(frozen=True)
class NominalPlatformParameters:
    """论文报告的标称平台参数。"""

    quadcopter_mass_kg: np.float64
    robotic_arm_mass_kg: np.float64
    wheelbase_m: np.float64
    robotic_arm_workspace_radius_m: np.float64


@dataclass(frozen=True)
class VariableGainParameters:
    """论文式 (4) 的变增益参数。"""

    w: np.float64
    d: np.float64


@dataclass(frozen=True)
class EnvelopeParameters:
    """三轴性能包络参数。"""

    rho_0: np.ndarray
    rho_inf: np.ndarray
    l: np.float64


@dataclass(frozen=True)
class PresetTrajectoryParameters:
    """论文式 (5) 的预设误差轨迹参数。"""

    c: np.ndarray


@dataclass(frozen=True)
class EsoParameters:
    """后续 M2 PR 使用的 ESO 论文参数。"""

    alpha: np.ndarray
    epsilon: np.ndarray


@dataclass(frozen=True)
class ControlParameters:
    """后续 M2 PR 使用的滑模控制增益。"""

    k_diagonal: np.ndarray
    lambda_diagonal: np.ndarray


@dataclass(frozen=True)
class ChannelParameters:
    """位置或姿态通道的论文参数。"""

    performance_envelope: EnvelopeParameters
    preset_error_trajectory: PresetTrajectoryParameters
    eso: EsoParameters
    control: ControlParameters


@dataclass(frozen=True)
class PaperParameters:
    """从 ``paper_default.yaml`` 读取的 M2 论文参数。"""

    paper: PaperIdentity
    nominal_platform: NominalPlatformParameters
    variable_gain: VariableGainParameters
    position: ChannelParameters
    attitude: ChannelParameters


def _resolve_paper_path(path: PathLike) -> Path:
    candidate = Path(path)
    if not candidate.is_absolute() and candidate == _DEFAULT_PAPER_PATH:
        return _REPO_ROOT / candidate
    return candidate


def _required(mapping: Mapping[str, Any], dotted_path: str) -> Any:
    current: Any = mapping
    traversed = []
    for key in dotted_path.split("."):
        traversed.append(key)
        if not isinstance(current, Mapping) or key not in current:
            raise ValueError(f"缺少参数字段：{'.'.join(traversed)}")
        current = current[key]
    return current


def _float64(value: Any, dotted_path: str) -> np.float64:
    try:
        converted = np.float64(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {dotted_path} 必须是数值") from exc
    return converted


def _vector3(value: Any, dotted_path: str) -> np.ndarray:
    try:
        converted = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"参数 {dotted_path} 必须是三轴数值") from exc
    if converted.shape != (3,):
        raise ValueError(f"参数 {dotted_path} 的形状必须为 (3,)")
    converted = converted.copy()
    converted.setflags(write=False)
    return converted


def _load_channel(raw: Mapping[str, Any], channel: str) -> ChannelParameters:
    prefix = channel
    envelope_prefix = f"{prefix}.performance_envelope"
    rho_0_key = "rho_0_m" if channel == "position" else "rho_0"
    rho_inf_key = "rho_inf_m" if channel == "position" else "rho_inf"
    envelope_parameters = EnvelopeParameters(
        rho_0=_vector3(
            _required(raw, f"{envelope_prefix}.{rho_0_key}"),
            f"{envelope_prefix}.{rho_0_key}",
        ),
        rho_inf=_vector3(
            _required(raw, f"{envelope_prefix}.{rho_inf_key}"),
            f"{envelope_prefix}.{rho_inf_key}",
        ),
        l=_float64(
            _required(raw, f"{envelope_prefix}.l_s_inv"),
            f"{envelope_prefix}.l_s_inv",
        ),
    )
    return ChannelParameters(
        performance_envelope=envelope_parameters,
        preset_error_trajectory=PresetTrajectoryParameters(
            c=_vector3(
                _required(raw, f"{prefix}.preset_error_trajectory.c"),
                f"{prefix}.preset_error_trajectory.c",
            )
        ),
        eso=EsoParameters(
            alpha=_vector3(
                _required(raw, f"{prefix}.eso.alpha_s_inv"),
                f"{prefix}.eso.alpha_s_inv",
            ),
            epsilon=_vector3(
                _required(raw, f"{prefix}.eso.epsilon"),
                f"{prefix}.eso.epsilon",
            ),
        ),
        control=ControlParameters(
            k_diagonal=_vector3(
                _required(raw, f"{prefix}.control.k_diagonal"),
                f"{prefix}.control.k_diagonal",
            ),
            lambda_diagonal=_vector3(
                _required(raw, f"{prefix}.control.lambda_diagonal"),
                f"{prefix}.control.lambda_diagonal",
            ),
        ),
    )


def load_paper_default(path: PathLike = "config/paper_default.yaml") -> PaperParameters:
    """读取并校验论文参数，数值只来自指定 YAML，不使用兜底默认值。

    依赖论文式 (4)、(5)、(7)、(9)、(14)、(21)；位置量沿 NED，姿态量
    沿 FRD/误差四元数接口继承。单位保持 YAML 中的 SI 约定（A-01）。
    """

    resolved_path = _resolve_paper_path(path)
    try:
        with resolved_path.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"无法读取论文参数文件：{resolved_path}") from exc
    if not isinstance(raw, Mapping):
        raise ValueError("论文参数文件根节点必须是映射")

    parameters = PaperParameters(
        paper=PaperIdentity(
            identifier=str(_required(raw, "paper.identifier")),
            revision=str(_required(raw, "paper.revision")),
            source_sha256=str(_required(raw, "paper.source_sha256")),
            formulas_frozen=_required(raw, "paper.formulas_frozen"),
            params_complete=_required(raw, "paper.params_complete"),
        ),
        nominal_platform=NominalPlatformParameters(
            quadcopter_mass_kg=_float64(
                _required(raw, "nominal_platform.quadcopter_mass_kg"),
                "nominal_platform.quadcopter_mass_kg",
            ),
            robotic_arm_mass_kg=_float64(
                _required(raw, "nominal_platform.robotic_arm_mass_kg"),
                "nominal_platform.robotic_arm_mass_kg",
            ),
            wheelbase_m=_float64(
                _required(raw, "nominal_platform.wheelbase_m"),
                "nominal_platform.wheelbase_m",
            ),
            robotic_arm_workspace_radius_m=_float64(
                _required(raw, "nominal_platform.robotic_arm_workspace_radius_m"),
                "nominal_platform.robotic_arm_workspace_radius_m",
            ),
        ),
        variable_gain=VariableGainParameters(
            w=_float64(_required(raw, "variable_gain.w"), "variable_gain.w"),
            d=_float64(_required(raw, "variable_gain.d"), "variable_gain.d"),
        ),
        position=_load_channel(raw, "position"),
        attitude=_load_channel(raw, "attitude"),
    )
    validate_parameters(parameters)
    return parameters


def paper_default_sha256(path: PathLike = "config/paper_default.yaml") -> str:
    """计算论文参数文件原始字节的 SHA-256，作为 M2 可追溯锚点。

    本函数不对应论文公式和物理量，因而无单位、坐标系或数值假设；摘要仅用于
    把后续黄金数据绑定到准确的 ``paper_default.yaml`` 字节内容。
    """

    resolved_path = _resolve_paper_path(path)
    digest = hashlib.sha256()
    try:
        with resolved_path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(65536), b""):
                digest.update(chunk)
    except OSError as exc:
        raise ValueError(f"无法读取论文参数文件：{resolved_path}") from exc
    return digest.hexdigest()


def _require_finite_scalar(value: np.float64, path: str) -> None:
    if not np.isfinite(value):
        raise ValueError(f"参数 {path} 必须为有限数")


def _require_finite_vector(value: np.ndarray, path: str) -> None:
    if value.shape != (3,):
        raise ValueError(f"参数 {path} 的形状必须为 (3,)")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"参数 {path} 必须全部为有限数")


def validate_parameters(parameters: PaperParameters) -> None:
    """校验 M2 所需论文参数；非法即抛出带字段路径的 ``ValueError``。

    覆盖论文式 (4)、(5)、(7)、(9)、(14)、(21) 的 SI 数值、NED/FRD 通道
    参数和 A-01 约定；本函数自身不产生带单位的计算结果。
    """

    if not isinstance(parameters, PaperParameters):
        raise ValueError("parameters 必须是 PaperParameters")
    if parameters.paper.formulas_frozen is not True:
        raise ValueError("参数 paper.formulas_frozen 必须为 true")
    if parameters.paper.params_complete is not True:
        raise ValueError("参数 paper.params_complete 必须为 true")
    source_hash = parameters.paper.source_sha256
    if len(source_hash) != 64 or any(character not in "0123456789abcdef" for character in source_hash.lower()):
        raise ValueError("参数 paper.source_sha256 必须是 64 位十六进制 SHA-256")

    platform = parameters.nominal_platform
    for name in (
        "quadcopter_mass_kg",
        "robotic_arm_mass_kg",
        "wheelbase_m",
        "robotic_arm_workspace_radius_m",
    ):
        value = getattr(platform, name)
        path = f"nominal_platform.{name}"
        _require_finite_scalar(value, path)
        if value <= 0.0:
            raise ValueError(f"参数 {path} 必须大于 0")

    gain = parameters.variable_gain
    _require_finite_scalar(gain.w, "variable_gain.w")
    _require_finite_scalar(gain.d, "variable_gain.d")
    if gain.w <= 0.0:
        raise ValueError("参数 variable_gain.w 必须大于 0")
    if gain.d < 0.0:
        raise ValueError("参数 variable_gain.d 必须大于等于 0")

    for channel_name in ("position", "attitude"):
        channel = getattr(parameters, channel_name)
        envelope_parameters = channel.performance_envelope
        envelope_prefix = f"{channel_name}.performance_envelope"
        _require_finite_vector(envelope_parameters.rho_0, f"{envelope_prefix}.rho_0")
        _require_finite_vector(envelope_parameters.rho_inf, f"{envelope_prefix}.rho_inf")
        _require_finite_scalar(envelope_parameters.l, f"{envelope_prefix}.l")
        if np.any(envelope_parameters.rho_inf <= 0.0):
            raise ValueError(f"参数 {envelope_prefix}.rho_inf 必须逐轴大于 0")
        if np.any(envelope_parameters.rho_0 <= envelope_parameters.rho_inf):
            raise ValueError(f"参数 {envelope_prefix}.rho_0 必须逐轴大于 rho_inf")
        if envelope_parameters.l <= 0.0:
            raise ValueError(f"参数 {envelope_prefix}.l 必须大于 0")

        c_path = f"{channel_name}.preset_error_trajectory.c"
        _require_finite_vector(channel.preset_error_trajectory.c, c_path)
        if np.any(channel.preset_error_trajectory.c <= 0.0):
            raise ValueError(f"参数 {c_path} 必须逐轴大于 0")

        alpha_path = f"{channel_name}.eso.alpha"
        epsilon_path = f"{channel_name}.eso.epsilon"
        _require_finite_vector(channel.eso.alpha, alpha_path)
        _require_finite_vector(channel.eso.epsilon, epsilon_path)
        if np.any(channel.eso.alpha <= 0.0):
            raise ValueError(f"参数 {alpha_path} 必须逐轴大于 0")
        if np.any((channel.eso.epsilon <= 0.0) | (channel.eso.epsilon >= 1.0)):
            raise ValueError(f"参数 {epsilon_path} 必须逐轴位于 (0,1)")

        for gain_name in ("k_diagonal", "lambda_diagonal"):
            gain_path = f"{channel_name}.control.{gain_name}"
            gain_vector = getattr(channel.control, gain_name)
            _require_finite_vector(gain_vector, gain_path)
            if np.any(gain_vector <= 0.0):
                raise ValueError(f"参数 {gain_path} 必须逐轴大于 0")


def variable_gain(e: Any, w: float, d: float):
    """计算论文式 (4) 的变增益 ``g(e)``。

    ``e`` 的单位和坐标继承被观测通道；论文直接使用 SI 数值（A-01）。
    小量斜率为 ``1/(w+d/2)``。实现只计算负指数，任意有限大误差均不会
    因 ``exp(abs(e))`` 溢出。标量返回 ``np.float64``，数组保持输入形状。
    """

    w_value = _float64(w, "w")
    d_value = _float64(d, "d")
    if not np.isfinite(w_value) or w_value <= 0.0:
        raise ValueError("参数 w 必须是大于 0 的有限数")
    if not np.isfinite(d_value) or d_value < 0.0:
        raise ValueError("参数 d 必须是大于等于 0 的有限数")
    try:
        error = np.asarray(e, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("参数 e 必须是有限数或有限数组") from exc
    if not np.all(np.isfinite(error)):
        raise ValueError("参数 e 必须全部为有限数")

    exp_negative = np.exp(-np.abs(error))
    sech = 2.0 * exp_negative / (1.0 + exp_negative * exp_negative)
    result = error / (w_value + 0.5 * d_value * sech)
    if result.ndim == 0:
        return np.float64(result)
    return np.asarray(result, dtype=np.float64)


# =============================================================================
# M2 PR2/PR3 扩展位置
# -----------------------------------------------------------------------------
# 变增益 ESO、RK4、样本时间戳和失效重置语义将在对应 PR 中实现。本 PR 不提供
# 占位类或伪输出，避免在黄金向量冻结前形成第二套未经验证的数学实现。
# =============================================================================
