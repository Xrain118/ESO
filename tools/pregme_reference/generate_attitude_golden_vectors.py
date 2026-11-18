"""生成 M2 姿态 ESO 合成夹具场景和可审计黄金向量。"""

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
import yaml

from .attitude import AttitudeEso
from .attitude import AttitudeEsoConfig
from .attitude import attitude_eso_config_from_paper
from .attitude import rigid_body_known_angular_acceleration
from .equations import PaperParameters
from .equations import load_paper_default
from .equations import paper_default_sha256


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SEED = 20261118
IMPLEMENTATION_BASELINE_COMMIT = "1c435abc513a55c279520095af67222cc2217db7"
SCENARIOS = (
    "composite",
    "zero",
    "known_torque",
    "constant",
    "step",
    "sine",
    "noise",
    "nonzero_initial",
)

CSV_COLUMNS = [
    "timestamp_us",
    "time_s",
    "angular_velocity_frd_rad_s_x",
    "angular_velocity_frd_rad_s_y",
    "angular_velocity_frd_rad_s_z",
    "torque_frd_n_m_x",
    "torque_frd_n_m_y",
    "torque_frd_n_m_z",
    "known_angular_acceleration_frd_rad_s2_x",
    "known_angular_acceleration_frd_rad_s2_y",
    "known_angular_acceleration_frd_rad_s2_z",
    "true_disturbance_frd_rad_s2_x",
    "true_disturbance_frd_rad_s2_y",
    "true_disturbance_frd_rad_s2_z",
    "h_omega_frd_rad_s_x",
    "h_omega_frd_rad_s_y",
    "h_omega_frd_rad_s_z",
    "innovation_frd_rad_s_x",
    "innovation_frd_rad_s_y",
    "innovation_frd_rad_s_z",
    "disturbance_hat_frd_rad_s2_x",
    "disturbance_hat_frd_rad_s2_y",
    "disturbance_hat_frd_rad_s2_z",
    "status",
    "integrated",
]


@dataclass(frozen=True)
class AttitudeFixtureContext:
    """姿态黄金数据使用的 fixture 身份、周期、惯量与溯源字段。"""

    fixture_id: str
    parameter_set_sha256: str
    watermark: str
    expected_attitude_period_s: np.float64
    minimum_period_s: np.float64
    maximum_period_s: np.float64
    inertia_kg_m2: np.ndarray
    inertia_frame: str
    inertia_source: str
    inertia_status: str
    inertia_replacement_condition: str
    inertia_failure_consequence: str
    file_sha256: str
    metadata_sha256: str


@dataclass(frozen=True)
class AttitudeScenario:
    """一个式 (14) 姿态 ESO 场景的输入、真值和参考输出。"""

    name: str
    seed: int
    timestamp_us: np.ndarray
    time_s: np.ndarray
    angular_velocity_frd_rad_s: np.ndarray
    torque_frd_n_m: np.ndarray
    known_angular_acceleration_frd_rad_s2: np.ndarray
    true_disturbance_frd_rad_s2: np.ndarray
    h_omega_frd_rad_s: np.ndarray
    innovation_frd_rad_s: np.ndarray
    disturbance_hat_frd_rad_s2: np.ndarray
    status: np.ndarray
    integrated: np.ndarray
    config: AttitudeEsoConfig
    fixture: AttitudeFixtureContext
    paper: PaperParameters


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_attitude_fixture_context(
    fixture_path: Path = REPO_ROOT / "config/m2_fixture.yaml",
    metadata_path: Path = REPO_ROOT / "config/m2_fixture_metadata.yaml",
) -> AttitudeFixtureContext:
    """读取并交叉校验 M2 姿态周期、合成惯量及其占位元数据。"""

    try:
        runtime = yaml.safe_load(fixture_path.read_text(encoding="utf-8"))
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        fixture = runtime["pregme_control"]["ros__parameters"]["fixture"]
        inertia_metadata = metadata["placeholders"]["inertia_matrix"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise ValueError("无法读取完整的 M2 姿态 fixture 与惯量元数据") from exc

    fixture_id = str(fixture["id"])
    parameter_hash = str(fixture["parameter_set_sha256"])
    watermark = str(fixture["watermark"])
    if metadata.get("fixture_id") != fixture_id:
        raise ValueError("M2 fixture_id 与 metadata 不一致")
    if metadata.get("parameter_set_sha256") != parameter_hash:
        raise ValueError("M2 fixture parameter_set_sha256 与 metadata 不一致")
    if metadata.get("watermark") != watermark:
        raise ValueError("M2 fixture watermark 与 metadata 不一致")
    if watermark != f"FIXTURE:{fixture_id}:{parameter_hash[:12]}":
        raise ValueError("M2 fixture watermark 与身份哈希不一致")

    inertia = np.diag(
        [
            fixture["inertia_xx_kg_m2"],
            fixture["inertia_yy_kg_m2"],
            fixture["inertia_zz_kg_m2"],
        ]
    ).astype(np.float64)
    metadata_inertia = np.asarray(inertia_metadata["value"], dtype=np.float64)
    if metadata_inertia.shape != (3, 3) or not np.array_equal(inertia, metadata_inertia):
        raise ValueError("M2 fixture 惯量与 metadata 不一致")
    rigid_body_known_angular_acceleration(np.zeros(3), np.zeros(3), inertia)
    inertia.setflags(write=False)

    values = {
        "expected_attitude_period_s": np.float64(
            fixture["expected_attitude_period_s"]
        ),
        "minimum_period_s": np.float64(fixture["minimum_period_s"]),
        "maximum_period_s": np.float64(fixture["maximum_period_s"]),
    }
    if not all(np.isfinite(value) and value > 0.0 for value in values.values()):
        raise ValueError("M2 fixture 姿态周期字段必须为有限正数")
    if not (
        values["minimum_period_s"]
        <= values["expected_attitude_period_s"]
        <= values["maximum_period_s"]
    ):
        raise ValueError("M2 fixture expected_attitude_period_s 超出允许窗口")
    if inertia_metadata.get("status") != "placeholder":
        raise ValueError("M2 fixture 惯量必须明确标记为 placeholder")

    return AttitudeFixtureContext(
        fixture_id=fixture_id,
        parameter_set_sha256=parameter_hash,
        watermark=watermark,
        expected_attitude_period_s=values["expected_attitude_period_s"],
        minimum_period_s=values["minimum_period_s"],
        maximum_period_s=values["maximum_period_s"],
        inertia_kg_m2=inertia,
        inertia_frame=str(inertia_metadata["frame"]),
        inertia_source=str(inertia_metadata["source"]),
        inertia_status=str(inertia_metadata["status"]),
        inertia_replacement_condition=str(
            inertia_metadata["replacement_condition"]
        ),
        inertia_failure_consequence=str(inertia_metadata["failure_consequence"]),
        file_sha256=_sha256(fixture_path),
        metadata_sha256=_sha256(metadata_path),
    )


def load_reference_context(
) -> Tuple[PaperParameters, AttitudeFixtureContext, AttitudeEsoConfig]:
    """从冻结 YAML 组装式 (14) 参数、姿态周期和合成惯量。"""

    paper = load_paper_default()
    fixture = load_attitude_fixture_context()
    config = attitude_eso_config_from_paper(
        paper, fixture.minimum_period_s, fixture.maximum_period_s
    )
    return paper, fixture, config


def _scenario_definition(
    name: str, time_s: np.ndarray, seed: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    samples = time_s.size
    initial_omega = np.zeros(3, dtype=np.float64)
    torque = np.zeros((samples, 3), dtype=np.float64)
    disturbance = np.zeros((samples, 3), dtype=np.float64)
    noise = np.zeros((samples, 3), dtype=np.float64)

    if name == "composite":
        initial_omega = np.array([0.12, -0.08, 0.05], dtype=np.float64)
        torque_mask = (time_s >= 1.0) & (time_s < 3.0)
        torque[torque_mask] = np.array([0.08, -0.06, 0.05])
        step_mask = (time_s >= 3.0) & (time_s < 8.0)
        disturbance[step_mask] = np.array([0.4, -0.25, 0.15])
        sine_mask = time_s >= 8.0
        phase_time = time_s[sine_mask] - 8.0
        base = np.array([0.4, -0.25, 0.15])
        amplitude = np.array([0.10, 0.08, 0.05])
        phase = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0])
        disturbance[sine_mask] = base + amplitude * np.sin(
            2.0 * np.pi * 0.5 * phase_time[:, None] + phase
        )
    elif name == "known_torque":
        torque[:] = np.array([0.08, -0.06, 0.05])
    elif name == "constant":
        disturbance[:] = np.array([0.0, 0.0, 1.0])
    elif name == "step":
        disturbance[time_s < 5.0] = np.array([0.0, 0.0, 0.5])
        disturbance[time_s >= 5.0] = np.array([0.0, 0.0, 1.5])
    elif name == "sine":
        amplitude = np.array([0.3, 0.2, 0.1])
        disturbance[:] = amplitude * np.sin(2.0 * np.pi * 0.01 * time_s[:, None])
    elif name == "noise":
        disturbance[:] = np.array([0.4, -0.25, 0.15])
        noise = np.random.default_rng(seed).normal(0.0, 1.0e-3, size=(samples, 3))
    elif name == "nonzero_initial":
        initial_omega = np.array([0.12, -0.08, 0.05], dtype=np.float64)
    elif name != "zero":
        raise ValueError(f"未知场景：{name}")
    return initial_omega, torque, disturbance, noise


def _scenario_duration_s(name: str) -> float:
    return {
        "composite": 12.0,
        "zero": 2.0,
        "known_torque": 5.0,
        "constant": 10.0,
        "step": 15.0,
        "sine": 120.0,
        "noise": 10.0,
        "nonzero_initial": 2.0,
    }[name]


def generate_attitude_scenario(
    name: str, seed: int = DEFAULT_SEED
) -> AttitudeScenario:
    """生成式 (13)/(14) 场景；真值欧拉推进独立于 ESO 的 RK4 内核。"""

    if name not in SCENARIOS:
        raise ValueError(f"未知场景：{name}")
    paper, fixture, config = load_reference_context()
    dt = fixture.expected_attitude_period_s
    sample_count = int(round(_scenario_duration_s(name) / float(dt))) + 1
    time_s = np.arange(sample_count, dtype=np.float64) * dt
    period_us = int(round(float(dt) * 1_000_000.0))
    timestamp_us = 1_000_000 + np.arange(sample_count, dtype=np.int64) * period_us
    initial_omega, torque, disturbance, noise = _scenario_definition(
        name, time_s, seed
    )

    true_omega = np.empty((sample_count, 3), dtype=np.float64)
    known_acceleration = np.empty_like(true_omega)
    true_omega[0] = initial_omega
    known_acceleration[0] = rigid_body_known_angular_acceleration(
        true_omega[0], torque[0], fixture.inertia_kg_m2
    )
    for index in range(1, sample_count):
        true_omega[index] = true_omega[index - 1] + dt * (
            known_acceleration[index - 1] + disturbance[index - 1]
        )
        known_acceleration[index] = rigid_body_known_angular_acceleration(
            true_omega[index], torque[index], fixture.inertia_kg_m2
        )
    measured_omega = true_omega + noise

    h = np.empty_like(measured_omega)
    innovation = np.empty_like(measured_omega)
    disturbance_hat = np.empty_like(measured_omega)
    status = np.empty(sample_count, dtype="<U16")
    integrated = np.empty(sample_count, dtype=np.bool_)
    observer = AttitudeEso(config)
    for index in range(sample_count):
        update = observer.update(
            measured_omega[index], known_acceleration[index], int(timestamp_us[index])
        )
        if not update.valid or update.estimate is None:
            raise RuntimeError(
                f"姿态场景 {name} 在样本 {index} 意外失效：{update.status.value}"
            )
        h[index] = update.estimate.h_omega_frd_rad_s
        innovation[index] = update.estimate.innovation_frd_rad_s
        disturbance_hat[index] = update.estimate.disturbance_hat_frd_rad_s2
        status[index] = update.status.value
        integrated[index] = update.integrated

    return AttitudeScenario(
        name=name,
        seed=seed,
        timestamp_us=timestamp_us,
        time_s=time_s,
        angular_velocity_frd_rad_s=measured_omega,
        torque_frd_n_m=torque,
        known_angular_acceleration_frd_rad_s2=known_acceleration,
        true_disturbance_frd_rad_s2=disturbance,
        h_omega_frd_rad_s=h,
        innovation_frd_rad_s=innovation,
        disturbance_hat_frd_rad_s2=disturbance_hat,
        status=status,
        integrated=integrated,
        config=config,
        fixture=fixture,
        paper=paper,
    )


def _float_text(value: float) -> str:
    return format(np.float64(value), ".17g")


def _row(scenario: AttitudeScenario, index: int) -> Sequence[str]:
    values = [str(int(scenario.timestamp_us[index])), _float_text(scenario.time_s[index])]
    for array in (
        scenario.angular_velocity_frd_rad_s,
        scenario.torque_frd_n_m,
        scenario.known_angular_acceleration_frd_rad_s2,
        scenario.true_disturbance_frd_rad_s2,
        scenario.h_omega_frd_rad_s,
        scenario.innovation_frd_rad_s,
        scenario.disturbance_hat_frd_rad_s2,
    ):
        values.extend(_float_text(value) for value in array[index])
    values.extend([scenario.status[index], "1" if scenario.integrated[index] else "0"])
    return values


def _manifest(scenario: AttitudeScenario, csv_path: Path) -> Dict[str, object]:
    source_paths = {
        "attitude": REPO_ROOT / "tools/pregme_reference/attitude.py",
        "shared_eso": REPO_ROOT / "tools/pregme_reference/eso.py",
        "generator": Path(__file__),
    }
    return {
        "schema_version": 1,
        "dataset_id": f"attitude_eso_{scenario.name}_v1",
        "scenario": scenario.name,
        "formula": ["(3)", "(13)", "(14)"],
        "frame": "FRD",
        "hold_semantics": "previous-sample zero-order hold on omega and u_omega",
        "estimate_semantics": "current omega minus updated h_omega",
        "truth_integration": "explicit Euler independent of observer RK4",
        "columns": CSV_COLUMNS,
        "units": {
            "timestamp_us": "us",
            "time_s": "s",
            "angular_velocity_frd_rad_s": "rad/s",
            "torque_frd_n_m": "N*m",
            "known_angular_acceleration_frd_rad_s2": "rad/s^2",
            "true_disturbance_frd_rad_s2": "rad/s^2",
            "h_omega_frd_rad_s": "rad/s",
            "innovation_frd_rad_s": "rad/s",
            "disturbance_hat_frd_rad_s2": "rad/s^2",
        },
        "sample_count": int(scenario.time_s.size),
        "dt_s": float(scenario.fixture.expected_attitude_period_s),
        "seed": int(scenario.seed),
        "random_noise_used": scenario.name == "noise",
        "parameters": {
            "alpha_omega": scenario.config.alpha.tolist(),
            "epsilon_omega": scenario.config.epsilon.tolist(),
            "w": float(scenario.config.w),
            "d": float(scenario.config.d),
            "minimum_period_s": float(scenario.config.minimum_period_s),
            "maximum_period_s": float(scenario.config.maximum_period_s),
        },
        "paper": {
            "identifier": scenario.paper.paper.identifier,
            "source_sha256": scenario.paper.paper.source_sha256,
            "paper_default_sha256": paper_default_sha256(),
        },
        "fixture": {
            "id": scenario.fixture.fixture_id,
            "parameter_set_sha256": scenario.fixture.parameter_set_sha256,
            "watermark": scenario.fixture.watermark,
            "file_sha256": scenario.fixture.file_sha256,
            "metadata_sha256": scenario.fixture.metadata_sha256,
        },
        "inertia": {
            "matrix_kg_m2": scenario.fixture.inertia_kg_m2.tolist(),
            "frame": scenario.fixture.inertia_frame,
            "source": scenario.fixture.inertia_source,
            "status": scenario.fixture.inertia_status,
            "replacement_condition": scenario.fixture.inertia_replacement_condition,
            "failure_consequence": scenario.fixture.inertia_failure_consequence,
        },
        "implementation_baseline_commit": IMPLEMENTATION_BASELINE_COMMIT,
        "source_sha256": {
            name: _sha256(path) for name, path in source_paths.items()
        },
        "csv_sha256": _sha256(csv_path),
        "python": "3.10.12",
        "numpy": np.__version__,
    }


def write_scenario(
    scenario: AttitudeScenario, output_dir: Path
) -> Tuple[Path, Path]:
    """以确定性 CSV/JSON 写出姿态场景并返回两个产物路径。"""

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = (
        "attitude_eso_v1"
        if scenario.name == "composite"
        else f"attitude_eso_{scenario.name}_v1"
    )
    csv_path = output_dir / f"{stem}.csv"
    manifest_path = output_dir / f"{stem}_manifest.json"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        for index in range(scenario.time_s.size):
            writer.writerow(_row(scenario, index))
    manifest_path.write_text(
        json.dumps(_manifest(scenario, csv_path), ensure_ascii=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return csv_path, manifest_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="composite")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--output-dir", type=Path, default=REPO_ROOT / "test/pregme/data"
    )
    args = parser.parse_args()
    scenario = generate_attitude_scenario(args.scenario, args.seed)
    csv_path, manifest_path = write_scenario(scenario, args.output_dir)
    print(f"CSV: {csv_path}")
    print(f"manifest: {manifest_path}")
    print(f"samples: {scenario.time_s.size}")
    print(f"baseline: {IMPLEMENTATION_BASELINE_COMMIT}")


if __name__ == "__main__":
    main()
