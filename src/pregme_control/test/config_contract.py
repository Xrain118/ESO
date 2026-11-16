#!/usr/bin/env python3
"""M1 配置契约校验。

检查三份参数文件是否仍然满足冻结的接口约定：
  1. config/onboard_default.yaml：安全默认值（Native、输出关闭、无夹具）
     以及八个 PX4 话题必须与冻结契约完全一致；
  2. config/paper_default.yaml：论文公式与参数完整性标志、缺失字段清单；
  3. config/m2_fixture.yaml + m2_fixture_metadata.yaml：规范化哈希、
     水印、回转半径自洽性；
  4. 最后比对 pregme_core/types.hpp 与 PregmeStatus.msg 的故障位是否逐位一致。

只读取文件、不做修改；任何一项不符都会以 AssertionError 失败，使 colcon test
返回非零。脚本由 CMake 以 --repo-root 参数调用。
"""

import argparse
import copy
import hashlib
import json
import math
import re
from pathlib import Path

import yaml


def load_yaml(path: Path):
    """读取 UTF-8 YAML 文件。"""
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def canonical_fixture_hash(runtime, metadata):
    """计算夹具参数集的规范化 SHA-256。

    先深拷贝并剔除哈希/水印自身（避免自引用），再把 runtime 与 metadata
    合成一个对象，用 sort_keys + 紧凑分隔符序列化为确定性 JSON，最后取
    SHA-256。注释、键顺序和空白都不影响结果，只对解析后的内容敏感。
    """
    runtime_copy = copy.deepcopy(runtime)
    fixture = runtime_copy["pregme_control"]["ros__parameters"]["fixture"]
    fixture.pop("parameter_set_sha256", None)
    fixture.pop("watermark", None)
    metadata_copy = copy.deepcopy(metadata)
    metadata_copy.pop("parameter_set_sha256", None)
    metadata_copy.pop("watermark", None)
    payload = json.dumps(
        {"runtime": runtime_copy, "metadata": metadata_copy},
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def require(condition, message):
    """断言辅助函数：失败时抛 AssertionError，并带上可读原因。"""
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.repo_root.resolve()

    # -------------------------------------------------------------------------
    # 1) 机上安全默认值：必须是 Native、输出关闭、不允许占位参数、无夹具，
    #    且八个话题与冻结契约逐字一致。这些是实机启动的最后一道安全底线。
    # -------------------------------------------------------------------------
    onboard = load_yaml(root / "config/onboard_default.yaml")["pregme_control"]["ros__parameters"]
    require(onboard["mode"] == 0, "onboard mode must default to native")
    require(onboard["enable_control_output"] is False, "onboard output must default off")
    require(onboard["allow_placeholder_in_sitl"] is False, "SITL placeholder permission must default off")
    require(onboard["fixture"]["has_placeholder"] is False, "onboard defaults must not contain fixtures")

    expected_topics = {
        "local_position": "/fmu/out/vehicle_local_position",
        "attitude": "/fmu/out/vehicle_attitude",
        "angular_velocity": "/fmu/out/vehicle_angular_velocity",
        "vehicle_status": "/fmu/out/vehicle_status_v1",
        "allocator_status": "/fmu/out/control_allocator_status",
        "reference": "/fmu/out/vehicle_local_position_setpoint",
        "native_thrust": "/fmu/out/vehicle_thrust_setpoint",
        "native_torque": "/fmu/out/vehicle_torque_setpoint",
    }
    require(onboard["topics"] == expected_topics, "onboard PX4 topic contract drifted")

    # -------------------------------------------------------------------------
    # 2) 论文默认参数：公式已冻结、报告参数齐全，缺失字段必须恰好是评审过的
    #    三项（惯量矩阵、控制器周期、饱和/导数策略），防止漏记或随口新增。
    # -------------------------------------------------------------------------
    paper = load_yaml(root / "config/paper_default.yaml")
    require(paper["paper"]["formulas_frozen"] is True, "M1 formulas must be frozen")
    require(paper["paper"]["params_complete"] is True, "reported paper parameters must be marked complete")
    require(
        set(paper["verification"]["missing"])
        == {"inertia_matrix", "control_period", "saturation_and_derivative_policy"},
        "paper missing-field list must contain exactly the three reviewed gaps",
    )

    # -------------------------------------------------------------------------
    # 3) M2 夹具：runtime 与 metadata 必须成对自洽。
    #    3.1 双方都声明含占位参数；
    #    3.2 规范化哈希一致（两份文件都未过期）；
    #    3.3 水印 == FIXTURE:<id>:<哈希前 12 位>，且两份文件相同；
    #    3.4 由质量与对角惯量反算的回转半径落在 [0.1, 0.4] m。
    # -------------------------------------------------------------------------
    runtime = load_yaml(root / "config/m2_fixture.yaml")
    metadata = load_yaml(root / "config/m2_fixture_metadata.yaml")
    fixture = runtime["pregme_control"]["ros__parameters"]["fixture"]
    require(fixture["has_placeholder"] is True, "M2 fixture must advertise placeholders")
    expected_hash = canonical_fixture_hash(runtime, metadata)
    require(fixture["parameter_set_sha256"] == expected_hash, "runtime fixture SHA-256 is stale")
    require(metadata["parameter_set_sha256"] == expected_hash, "metadata fixture SHA-256 is stale")
    require(
        metadata["watermark"] == f"FIXTURE:{fixture['id']}:{expected_hash[:12]}",
        "fixture watermark is stale",
    )
    require(
        fixture["watermark"] == metadata["watermark"],
        "runtime and metadata fixture watermarks differ",
    )
    mass = fixture["body_mass_kg"]
    inertia = [
        fixture["inertia_xx_kg_m2"],
        fixture["inertia_yy_kg_m2"],
        fixture["inertia_zz_kg_m2"],
    ]
    radii = [math.sqrt(value / mass) for value in inertia]
    require(all(0.1 <= value <= 0.4 for value in radii), "fixture radius of gyration is inconsistent")

    # -------------------------------------------------------------------------
    # 4) 故障位一致性：从 C++ 枚举和 ROS 消息中分别解析位号，按下面的
    #    name_map 逐项比对。任何一侧新增/改号而未同步，都会在这里失败。
    # -------------------------------------------------------------------------
    core_text = (root / "src/pregme_core/include/pregme_core/types.hpp").read_text(encoding="utf-8")
    msg_text = (root / "src/eso_interfaces/msg/PregmeStatus.msg").read_text(encoding="utf-8")
    core_faults = {
        name: 1 << int(bit)
        for name, bit in re.findall(r"(Fault[A-Za-z]+)\s*=\s*1U\s*<<\s*(\d+)", core_text)
    }
    msg_faults = {
        name: int(value)
        for name, value in re.findall(r"uint32\s+(FAULT_[A-Z_]+)\s*=\s*(\d+)", msg_text)
    }
    name_map = {
        "FaultInvalidMode": "FAULT_INVALID_MODE",
        "FaultLocalPosition": "FAULT_LOCAL_POSITION",
        "FaultAttitude": "FAULT_ATTITUDE",
        "FaultAngularVelocity": "FAULT_ANGULAR_VELOCITY",
        "FaultVehicleStatus": "FAULT_VEHICLE_STATUS",
        "FaultAllocatorStatus": "FAULT_ALLOCATOR_STATUS",
        "FaultTimestamp": "FAULT_TIMESTAMP",
        "FaultEstimatorReset": "FAULT_ESTIMATOR_RESET",
        "FaultNonFinite": "FAULT_NON_FINITE",
        "FaultAlgorithmUnavailable": "FAULT_ALGORITHM_UNAVAILABLE",
        "FaultOutputDisabled": "FAULT_OUTPUT_DISABLED",
        "FaultReference": "FAULT_REFERENCE",
        "FaultNativeThrust": "FAULT_NATIVE_THRUST",
        "FaultNativeTorque": "FAULT_NATIVE_TORQUE",
        "FaultParameterInvalid": "FAULT_PARAMETER_INVALID",
        "FaultPlaceholderParameters": "FAULT_PLACEHOLDER_PARAMETERS",
        "FaultSimulationContext": "FAULT_SIMULATION_CONTEXT",
        "FaultNotArmed": "FAULT_NOT_ARMED",
        "FaultNotOffboard": "FAULT_NOT_OFFBOARD",
        "FaultFailsafe": "FAULT_FAILSAFE",
        "FaultReentryRequired": "FAULT_REENTRY_REQUIRED",
    }
    require(set(core_faults) == set(name_map), "unexpected core fault enum set")
    for core_name, msg_name in name_map.items():
        require(msg_faults.get(msg_name) == core_faults[core_name], f"fault bit drift: {core_name}")

    print("M1 config contract: ok")


if __name__ == "__main__":
    main()
