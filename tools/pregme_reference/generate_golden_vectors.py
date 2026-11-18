"""生成 M2 位置 ESO 场景和可审计黄金向量。"""

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np
import yaml

from .equations import PaperParameters
from .equations import load_paper_default
from .equations import paper_default_sha256
from .eso import EsoUpdateStatus
from .eso import PositionEso
from .eso import PositionEsoConfig
from .eso import position_eso_config_from_paper


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SEED = 20261118
IMPLEMENTATION_BASELINE_COMMIT = "fdc34c3e557233bde199d1af23e7ac857701a1a9"
SCENARIOS = (
    "composite",
    "zero",
    "known_input",
    "constant",
    "step",
    "sine",
    "noise",
    "nonzero_initial",
)

CSV_COLUMNS = [
    "timestamp_us",
    "time_s",
    "velocity_ned_m_s_x",
    "velocity_ned_m_s_y",
    "velocity_ned_m_s_z",
    "known_acceleration_ned_m_s2_x",
    "known_acceleration_ned_m_s2_y",
    "known_acceleration_ned_m_s2_z",
    "true_disturbance_ned_m_s2_x",
    "true_disturbance_ned_m_s2_y",
    "true_disturbance_ned_m_s2_z",
    "h_v_ned_m_s_x",
    "h_v_ned_m_s_y",
    "h_v_ned_m_s_z",
    "innovation_ned_m_s_x",
    "innovation_ned_m_s_y",
    "innovation_ned_m_s_z",
    "disturbance_hat_ned_m_s2_x",
    "disturbance_hat_ned_m_s2_y",
    "disturbance_hat_ned_m_s2_z",
    "status",
    "integrated",
]


@dataclass(frozen=True)
class FixtureContext:
    """黄金数据所需的 M2 fixture 身份和周期字段。"""

    fixture_id: str
    parameter_set_sha256: str
    watermark: str
    expected_position_period_s: np.float64
    minimum_period_s: np.float64
    maximum_period_s: np.float64
    file_sha256: str


@dataclass(frozen=True)
class PositionScenario:
    """一个位置 ESO 场景的全部输入、真值与参考输出。"""

    name: str
    seed: int
    timestamp_us: np.ndarray
    time_s: np.ndarray
    velocity_ned_m_s: np.ndarray
    known_acceleration_ned_m_s2: np.ndarray
    true_disturbance_ned_m_s2: np.ndarray
    h_v_ned_m_s: np.ndarray
    innovation_ned_m_s: np.ndarray
    disturbance_hat_ned_m_s2: np.ndarray
    status: np.ndarray
    integrated: np.ndarray
    config: PositionEsoConfig
    fixture: FixtureContext
    paper: PaperParameters


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_fixture_context(
    fixture_path: Path = REPO_ROOT / "config/m2_fixture.yaml",
    metadata_path: Path = REPO_ROOT / "config/m2_fixture_metadata.yaml",
) -> FixtureContext:
    """读取并交叉校验 M2 fixture 身份及位置周期，不提供默认数值。"""

    try:
        with fixture_path.open(encoding="utf-8") as stream:
            runtime = yaml.safe_load(stream)
        with metadata_path.open(encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        fixture = runtime["pregme_control"]["ros__parameters"]["fixture"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise ValueError("无法读取完整的 M2 fixture 身份和周期字段") from exc

    fixture_id = str(fixture["id"])
    parameter_hash = str(fixture["parameter_set_sha256"])
    watermark = str(fixture["watermark"])
    expected_watermark = f"FIXTURE:{fixture_id}:{parameter_hash[:12]}"
    if metadata.get("fixture_id") != fixture_id:
        raise ValueError("M2 fixture_id 与 metadata 不一致")
    if metadata.get("parameter_set_sha256") != parameter_hash:
        raise ValueError("M2 fixture parameter_set_sha256 与 metadata 不一致")
    if metadata.get("watermark") != watermark or watermark != expected_watermark:
        raise ValueError("M2 fixture watermark 与身份哈希不一致")

    values = {
        "expected_position_period_s": np.float64(
            fixture["expected_position_period_s"]
        ),
        "minimum_period_s": np.float64(fixture["minimum_period_s"]),
        "maximum_period_s": np.float64(fixture["maximum_period_s"]),
    }
    if not all(np.isfinite(value) and value > 0.0 for value in values.values()):
        raise ValueError("M2 fixture 位置周期字段必须为有限正数")
    if not (
        values["minimum_period_s"]
        <= values["expected_position_period_s"]
        <= values["maximum_period_s"]
    ):
        raise ValueError("M2 fixture expected_position_period_s 超出允许窗口")

    return FixtureContext(
        fixture_id=fixture_id,
        parameter_set_sha256=parameter_hash,
        watermark=watermark,
        expected_position_period_s=values["expected_position_period_s"],
        minimum_period_s=values["minimum_period_s"],
        maximum_period_s=values["maximum_period_s"],
        file_sha256=_sha256(fixture_path),
    )


def load_reference_context() -> Tuple[PaperParameters, FixtureContext, PositionEsoConfig]:
    """从冻结 YAML 组装位置 ESO 论文参数和 M2 周期窗口。"""

    paper = load_paper_default()
    fixture = load_fixture_context()
    config = position_eso_config_from_paper(
        paper,
        fixture.minimum_period_s,
        fixture.maximum_period_s,
    )
    return paper, fixture, config


def _scenario_definition(
    name: str, time_s: np.ndarray, seed: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    samples = time_s.size
    initial_velocity = np.zeros(3, dtype=np.float64)
    known_input = np.zeros((samples, 3), dtype=np.float64)
    disturbance = np.zeros((samples, 3), dtype=np.float64)
    noise = np.zeros((samples, 3), dtype=np.float64)

    if name == "composite":
        initial_velocity = np.array([0.3, -0.2, 0.1], dtype=np.float64)
        known_mask = (time_s >= 1.0) & (time_s < 3.0)
        known_input[known_mask] = np.array([0.2, -0.1, 0.05])
        step_mask = (time_s >= 3.0) & (time_s < 8.0)
        disturbance[step_mask] = np.array([0.4, -0.25, 0.15])
        sine_mask = time_s >= 8.0
        phase_time = time_s[sine_mask] - 8.0
        base = np.array([0.4, -0.25, 0.15])
        amplitude = np.array([0.1, 0.08, 0.05])
        phase = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0])
        disturbance[sine_mask] = base + amplitude * np.sin(
            2.0 * np.pi * 0.5 * phase_time[:, None] + phase
        )
    elif name == "known_input":
        known_input[:] = np.array([0.2, -0.1, 0.05])
    elif name == "constant":
        disturbance[:] = 1.0
    elif name == "step":
        disturbance[time_s < 5.0] = 0.5
        disturbance[time_s >= 5.0] = 1.5
    elif name == "sine":
        amplitude = np.array([0.3, 0.2, 0.1])
        disturbance[:] = amplitude * np.sin(2.0 * np.pi * 0.01 * time_s[:, None])
    elif name == "noise":
        disturbance[:] = np.array([0.4, -0.25, 0.15])
        noise = np.random.default_rng(seed).normal(0.0, 1.0e-3, size=(samples, 3))
    elif name == "nonzero_initial":
        initial_velocity = np.array([0.3, -0.2, 0.1], dtype=np.float64)
    elif name != "zero":
        raise ValueError(f"未知场景：{name}")
    return initial_velocity, known_input, disturbance, noise


def _scenario_duration_s(name: str) -> float:
    return {
        "composite": 12.0,
        "zero": 2.0,
        "known_input": 5.0,
        "constant": 60.0,
        "step": 15.0,
        "sine": 120.0,
        "noise": 10.0,
        "nonzero_initial": 2.0,
    }[name]


def generate_position_scenario(
    name: str,
    seed: int = DEFAULT_SEED,
) -> PositionScenario:
    """生成指定式 (7) 场景；随机性只来自显式 ``seed``。"""

    if name not in SCENARIOS:
        raise ValueError(f"未知场景：{name}")
    paper, fixture, config = load_reference_context()
    dt = fixture.expected_position_period_s
    duration_s = _scenario_duration_s(name)
    sample_count = int(round(duration_s / float(dt))) + 1
    time_s = np.arange(sample_count, dtype=np.float64) * dt
    period_us = int(round(float(dt) * 1_000_000.0))
    timestamp_us = 1_000_000 + np.arange(sample_count, dtype=np.int64) * period_us
    initial_velocity, known_input, disturbance, noise = _scenario_definition(
        name, time_s, seed
    )

    true_velocity = np.empty((sample_count, 3), dtype=np.float64)
    true_velocity[0] = initial_velocity
    for index in range(1, sample_count):
        true_velocity[index] = true_velocity[index - 1] + dt * (
            known_input[index - 1] + disturbance[index - 1]
        )
    measured_velocity = true_velocity + noise

    h = np.empty_like(measured_velocity)
    innovation = np.empty_like(measured_velocity)
    disturbance_hat = np.empty_like(measured_velocity)
    status = np.empty(sample_count, dtype="<U16")
    integrated = np.empty(sample_count, dtype=np.bool_)
    observer = PositionEso(config)
    for index in range(sample_count):
        update = observer.update(
            measured_velocity[index], known_input[index], int(timestamp_us[index])
        )
        if not update.valid or update.estimate is None:
            raise RuntimeError(f"场景 {name} 在样本 {index} 意外失效：{update.status.value}")
        h[index] = update.estimate.h_v_ned_m_s
        innovation[index] = update.estimate.innovation_ned_m_s
        disturbance_hat[index] = update.estimate.disturbance_hat_ned_m_s2
        status[index] = update.status.value
        integrated[index] = update.integrated

    return PositionScenario(
        name=name,
        seed=seed,
        timestamp_us=timestamp_us,
        time_s=time_s,
        velocity_ned_m_s=measured_velocity,
        known_acceleration_ned_m_s2=known_input,
        true_disturbance_ned_m_s2=disturbance,
        h_v_ned_m_s=h,
        innovation_ned_m_s=innovation,
        disturbance_hat_ned_m_s2=disturbance_hat,
        status=status,
        integrated=integrated,
        config=config,
        fixture=fixture,
        paper=paper,
    )


def _float_text(value: float) -> str:
    return format(np.float64(value), ".17g")


def _row(scenario: PositionScenario, index: int) -> Sequence[str]:
    values = [
        str(int(scenario.timestamp_us[index])),
        _float_text(scenario.time_s[index]),
    ]
    for array in (
        scenario.velocity_ned_m_s,
        scenario.known_acceleration_ned_m_s2,
        scenario.true_disturbance_ned_m_s2,
        scenario.h_v_ned_m_s,
        scenario.innovation_ned_m_s,
        scenario.disturbance_hat_ned_m_s2,
    ):
        values.extend(_float_text(value) for value in array[index])
    values.extend(
        [scenario.status[index], "1" if scenario.integrated[index] else "0"]
    )
    return values


def _manifest(scenario: PositionScenario, csv_path: Path) -> Dict[str, object]:
    generator_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    return {
        "schema_version": 1,
        "dataset_id": f"position_eso_{scenario.name}_v1",
        "scenario": scenario.name,
        "formula": ["(3)", "(7)"],
        "frame": "NED",
        "hold_semantics": "previous-sample zero-order hold on y and u",
        "estimate_semantics": "current y minus updated h",
        "columns": CSV_COLUMNS,
        "units": {
            "timestamp_us": "us",
            "time_s": "s",
            "velocity_ned_m_s": "m/s",
            "known_acceleration_ned_m_s2": "m/s^2",
            "true_disturbance_ned_m_s2": "m/s^2",
            "h_v_ned_m_s": "m/s",
            "innovation_ned_m_s": "m/s",
            "disturbance_hat_ned_m_s2": "m/s^2",
        },
        "sample_count": int(scenario.time_s.size),
        "dt_s": float(scenario.fixture.expected_position_period_s),
        "seed": int(scenario.seed),
        "random_noise_used": scenario.name == "noise",
        "parameters": {
            "alpha": scenario.config.alpha.tolist(),
            "epsilon": scenario.config.epsilon.tolist(),
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
        },
        "implementation_baseline_commit": IMPLEMENTATION_BASELINE_COMMIT,
        "generator_source_sha256": generator_sha256,
        "csv_sha256": _sha256(csv_path),
        "python": "3.10.12",
        "numpy": np.__version__,
    }


def write_scenario(
    scenario: PositionScenario,
    output_dir: Path,
) -> Tuple[Path, Path]:
    """以确定性 CSV 和 JSON 写出场景，返回两个产物路径。"""

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = "position_eso_v1" if scenario.name == "composite" else f"position_eso_{scenario.name}_v1"
    csv_path = output_dir / f"{stem}.csv"
    manifest_path = output_dir / f"{stem}_manifest.json"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        for index in range(scenario.time_s.size):
            writer.writerow(_row(scenario, index))
    manifest = _manifest(scenario, csv_path)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return csv_path, manifest_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="composite")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPO_ROOT / "test/pregme/data",
    )
    args = parser.parse_args()
    scenario = generate_position_scenario(args.scenario, args.seed)
    csv_path, manifest_path = write_scenario(scenario, args.output_dir)
    print(f"CSV: {csv_path}")
    print(f"manifest: {manifest_path}")
    print(f"samples: {scenario.time_s.size}")
    print(f"baseline: {IMPLEMENTATION_BASELINE_COMMIT}")


if __name__ == "__main__":
    main()
