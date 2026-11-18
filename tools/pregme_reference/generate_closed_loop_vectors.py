"""生成 M2 T06 位置/姿态合成闭环时序黄金向量。"""

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .attitude import attitude_eso_config_from_paper
from .attitude import rigid_body_known_angular_acceleration
from .control import attitude_control_torque
from .control import attitude_error_terms
from .control import position_control_config_from_paper
from .control import position_control_force
from .control import position_error_terms
from .control import position_known_acceleration
from .envelope import envelope
from .envelope import preset_trajectory
from .envelope import preset_trajectory_ddot
from .envelope import preset_trajectory_dot
from .equations import PaperParameters
from .equations import load_paper_default
from .equations import paper_default_sha256
from .eso import position_eso_config_from_paper
from .eso import variable_gain_eso_estimate
from .eso import variable_gain_eso_rk4_step
from .generate_attitude_golden_vectors import AttitudeFixtureContext
from .generate_attitude_golden_vectors import load_attitude_fixture_context
from .generate_golden_vectors import load_fixture_context


REPO_ROOT = Path(__file__).resolve().parents[2]
IMPLEMENTATION_BASELINE_COMMIT = "9472e85c7fe39b1d71c893ef53f6630922f9cd84"
SCENARIO_ID = "synthetic_closed_loop_v1"
SAMPLE_COUNT = 1601
START_TIMESTAMP_US = 1_000_000


def _axis_columns(prefix: str) -> List[str]:
    return [f"{prefix}_{axis}" for axis in ("x", "y", "z")]


def _matrix_columns(prefix: str) -> List[str]:
    return [f"{prefix}_r{row}c{column}" for row in range(3) for column in range(3)]


POSITION_COLUMNS = (
    ["timestamp_us", "time_s"]
    + _axis_columns("position_ned_m")
    + _axis_columns("desired_position_ned_m")
    + _axis_columns("velocity_ned_m_s")
    + _axis_columns("desired_velocity_ned_m_s")
    + _axis_columns("desired_acceleration_ned_m_s2")
    + _axis_columns("true_disturbance_ned_m_s2")
    + _axis_columns("disturbance_hat_ned_m_s2")
    + _axis_columns("h_v_ned_m_s")
    + _axis_columns("innovation_ned_m_s")
    + _axis_columns("rho_ned_m")
    + _axis_columns("beta_ned_m")
    + _axis_columns("beta_dot_ned_m_s")
    + _axis_columns("beta_ddot_ned_m_s2")
    + _axis_columns("tracking_error_ned_m")
    + _axis_columns("z_ned_m")
    + _axis_columns("z_dot_ned_m_s")
    + _axis_columns("sliding_ned_m_s")
    + _axis_columns("force_ned_n")
    + _axis_columns("known_acceleration_ned_m_s2")
    + ["status", "integrated"]
)

ATTITUDE_COLUMNS = (
    ["timestamp_us", "time_s"]
    + _matrix_columns("rotation_ned_from_frd")
    + _axis_columns("angular_velocity_frd_rad_s")
    + _axis_columns("true_disturbance_frd_rad_s2")
    + _axis_columns("disturbance_hat_frd_rad_s2")
    + _axis_columns("h_omega_frd_rad_s")
    + _axis_columns("innovation_frd_rad_s")
    + _axis_columns("rho")
    + _axis_columns("beta")
    + _axis_columns("beta_dot_s_inv")
    + _axis_columns("beta_ddot_s_inv2")
    + ["q0"]
    + _axis_columns("qv")
    + _axis_columns("z")
    + _axis_columns("z_dot_s_inv")
    + _axis_columns("sliding_s_inv")
    + _axis_columns("nonlinear_term_frd_rad_s2")
    + _axis_columns("torque_frd_n_m")
    + _axis_columns("known_angular_acceleration_frd_rad_s2")
    + ["status", "integrated"]
)


@dataclass(frozen=True)
class ClosedLoopData:
    """闭环 CSV 行、指标和可追溯上下文。"""

    position_rows: List[List[str]]
    attitude_rows: List[List[str]]
    metrics: Dict[str, object]
    paper: PaperParameters
    fixture: AttitudeFixtureContext


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _float_text(value: float) -> str:
    return format(np.float64(value), ".17g")


def _position_disturbance(time_s: float) -> np.ndarray:
    if time_s < 2.0:
        return np.zeros(3, dtype=np.float64)
    base = np.array([0.35, -0.25, 0.18], dtype=np.float64)
    if time_s < 8.0:
        return base
    amplitude = np.array([0.12, 0.10, 0.08], dtype=np.float64)
    phase = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0])
    return base + amplitude * np.sin(
        2.0 * np.pi * 0.5 * (time_s - 8.0) + phase
    )


def _attitude_disturbance(time_s: float) -> np.ndarray:
    if time_s < 2.0:
        return np.zeros(3, dtype=np.float64)
    base = np.array([0.18, -0.12, 0.10], dtype=np.float64)
    if time_s < 8.0:
        return base
    amplitude = np.array([0.06, 0.05, 0.04], dtype=np.float64)
    phase = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0])
    return base + amplitude * np.sin(
        2.0 * np.pi * 0.5 * (time_s - 8.0) + phase
    )


def _so3_exponential(rotation_vector: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(rotation_vector)
    if angle < 1.0e-15:
        return np.eye(3, dtype=np.float64)
    axis = rotation_vector / angle
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return (
        np.eye(3, dtype=np.float64)
        + np.sin(angle) * cross
        + (1.0 - np.cos(angle)) * (cross @ cross)
    )


def _attitude_rk4_step(
    angular_velocity: np.ndarray,
    torque: np.ndarray,
    disturbance: np.ndarray,
    inertia: np.ndarray,
    dt: np.float64,
) -> np.ndarray:
    def derivative(stage: np.ndarray) -> np.ndarray:
        return (
            np.linalg.solve(
                inertia,
                torque - np.cross(stage, inertia @ stage),
            )
            + disturbance
        )

    k1 = derivative(angular_velocity)
    k2 = derivative(angular_velocity + 0.5 * dt * k1)
    k3 = derivative(angular_velocity + 0.5 * dt * k2)
    k4 = derivative(angular_velocity + dt * k3)
    return angular_velocity + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def _extend(row: List[str], *arrays: np.ndarray) -> None:
    for array in arrays:
        row.extend(_float_text(value) for value in np.asarray(array).reshape(-1))


def generate_closed_loop_data() -> ClosedLoopData:
    """生成完整闭环时序；不使用随机数或论文未报告的实验数据。"""

    paper = load_paper_default()
    position_fixture = load_fixture_context()
    fixture = load_attitude_fixture_context()
    dt = position_fixture.expected_position_period_s
    if dt != fixture.expected_attitude_period_s:
        raise ValueError("闭环复现要求位置与姿态 fixture 周期一致")
    period_us = int(round(float(dt) * 1_000_000.0))
    time_s = np.arange(SAMPLE_COUNT, dtype=np.float64) * dt
    timestamps = START_TIMESTAMP_US + np.arange(SAMPLE_COUNT) * period_us
    zero = np.zeros(3, dtype=np.float64)

    position_config = position_control_config_from_paper(paper)
    position_eso = position_eso_config_from_paper(
        paper, fixture.minimum_period_s, fixture.maximum_period_s
    )
    desired_position = np.array([0.8, -0.8, -0.7], dtype=np.float64)
    position = np.array([0.0, 0.0, -1.5], dtype=np.float64)
    velocity = zero.copy()
    beta0_position = position - desired_position
    h_position = velocity.copy()
    held_velocity = velocity.copy()
    held_position_input = zero.copy()
    position_rows: List[List[str]] = []
    position_margins: List[float] = []
    position_errors: List[np.ndarray] = []
    position_true_disturbances: List[np.ndarray] = []
    position_estimates: List[np.ndarray] = []
    position_forces: List[np.ndarray] = []

    attitude_eso = attitude_eso_config_from_paper(
        paper, fixture.minimum_period_s, fixture.maximum_period_s
    )
    desired_rotation = np.eye(3, dtype=np.float64)
    rotation = _so3_exponential(np.array([0.3, 0.0, 0.0]))
    angular_velocity = zero.copy()
    initial_attitude = attitude_error_terms(
        rotation,
        desired_rotation,
        angular_velocity,
        zero,
        zero,
        zero,
        paper.attitude.control.lambda_diagonal,
    )
    beta0_attitude = initial_attitude.qv.copy()
    h_attitude = angular_velocity.copy()
    held_angular_velocity = angular_velocity.copy()
    held_attitude_input = zero.copy()
    attitude_rows: List[List[str]] = []
    attitude_margins: List[float] = []
    attitude_errors: List[np.ndarray] = []
    attitude_true_disturbances: List[np.ndarray] = []
    attitude_estimates: List[np.ndarray] = []
    attitude_torques: List[np.ndarray] = []
    orthogonality_errors: List[float] = []
    determinant_errors: List[float] = []

    for index, current_time in enumerate(time_s):
        integrated = index > 0
        status = "integrated" if integrated else "initialized"

        if integrated:
            h_position = variable_gain_eso_rk4_step(
                h_position,
                held_velocity,
                held_position_input,
                position_eso.alpha,
                position_eso.epsilon,
                position_eso.w,
                position_eso.d,
                dt,
            )
            position_innovation, position_estimate = variable_gain_eso_estimate(
                h_position,
                velocity,
                position_eso.alpha,
                position_eso.epsilon,
                position_eso.w,
                position_eso.d,
            )
        else:
            position_innovation = zero.copy()
            position_estimate = zero.copy()

        position_trajectory_args = (
            current_time,
            beta0_position,
            zero,
            paper.position.performance_envelope.l,
            paper.position.preset_error_trajectory.c,
        )
        position_beta = preset_trajectory(*position_trajectory_args)
        position_beta_dot = preset_trajectory_dot(*position_trajectory_args)
        position_beta_ddot = preset_trajectory_ddot(*position_trajectory_args)
        position_rho = envelope(
            current_time,
            paper.position.performance_envelope.rho_0,
            paper.position.performance_envelope.rho_inf,
            paper.position.performance_envelope.l,
        )
        position_terms = position_error_terms(
            position,
            velocity,
            desired_position,
            zero,
            position_beta,
            position_beta_dot,
            position_config.lambda_,
        )
        force = position_control_force(
            position_config,
            position_terms,
            zero,
            position_beta_ddot,
            position_estimate,
        )
        position_input = position_known_acceleration(
            force, position_config.total_mass_kg
        )
        position_disturbance = _position_disturbance(float(current_time))
        position_row = [str(int(timestamps[index])), _float_text(current_time)]
        _extend(
            position_row,
            position,
            desired_position,
            velocity,
            zero,
            zero,
            position_disturbance,
            position_estimate,
            h_position,
            position_innovation,
            position_rho,
            position_beta,
            position_beta_dot,
            position_beta_ddot,
            position_terms.tracking_error_ned_m,
            position_terms.z_ned_m,
            position_terms.z_dot_ned_m_s,
            position_terms.sliding_ned_m_s,
            force,
            position_input,
        )
        position_row.extend([status, "1" if integrated else "0"])
        position_rows.append(position_row)
        position_margins.append(
            float(
                np.min(
                    position_rho
                    - np.abs(position_terms.tracking_error_ned_m)
                )
            )
        )
        position_errors.append(position_terms.tracking_error_ned_m.copy())
        position_true_disturbances.append(position_disturbance.copy())
        position_estimates.append(position_estimate.copy())
        position_forces.append(force.copy())

        if integrated:
            h_attitude = variable_gain_eso_rk4_step(
                h_attitude,
                held_angular_velocity,
                held_attitude_input,
                attitude_eso.alpha,
                attitude_eso.epsilon,
                attitude_eso.w,
                attitude_eso.d,
                dt,
            )
            attitude_innovation, attitude_estimate = variable_gain_eso_estimate(
                h_attitude,
                angular_velocity,
                attitude_eso.alpha,
                attitude_eso.epsilon,
                attitude_eso.w,
                attitude_eso.d,
            )
        else:
            attitude_innovation = zero.copy()
            attitude_estimate = zero.copy()

        attitude_trajectory_args = (
            current_time,
            beta0_attitude,
            zero,
            paper.attitude.performance_envelope.l,
            paper.attitude.preset_error_trajectory.c,
        )
        attitude_beta = preset_trajectory(*attitude_trajectory_args)
        attitude_beta_dot = preset_trajectory_dot(*attitude_trajectory_args)
        attitude_beta_ddot = preset_trajectory_ddot(*attitude_trajectory_args)
        attitude_rho = envelope(
            current_time,
            paper.attitude.performance_envelope.rho_0,
            paper.attitude.performance_envelope.rho_inf,
            paper.attitude.performance_envelope.l,
        )
        attitude_terms = attitude_error_terms(
            rotation,
            desired_rotation,
            angular_velocity,
            zero,
            attitude_beta,
            attitude_beta_dot,
            paper.attitude.control.lambda_diagonal,
        )
        attitude_control = attitude_control_torque(
            fixture.inertia_kg_m2,
            angular_velocity,
            attitude_terms,
            zero,
            zero,
            attitude_beta_ddot,
            attitude_estimate,
            paper.attitude.control.lambda_diagonal,
            paper.attitude.control.k_diagonal,
        )
        attitude_input = rigid_body_known_angular_acceleration(
            angular_velocity,
            attitude_control.torque_frd_n_m,
            fixture.inertia_kg_m2,
        )
        attitude_disturbance = _attitude_disturbance(float(current_time))
        attitude_row = [str(int(timestamps[index])), _float_text(current_time)]
        _extend(
            attitude_row,
            rotation,
            angular_velocity,
            attitude_disturbance,
            attitude_estimate,
            h_attitude,
            attitude_innovation,
            attitude_rho,
            attitude_beta,
            attitude_beta_dot,
            attitude_beta_ddot,
        )
        attitude_row.append(_float_text(attitude_terms.q0))
        _extend(
            attitude_row,
            attitude_terms.qv,
            attitude_terms.z,
            attitude_terms.z_dot,
            attitude_terms.sliding,
            attitude_control.nonlinear_term_frd_rad_s2,
            attitude_control.torque_frd_n_m,
            attitude_input,
        )
        attitude_row.extend([status, "1" if integrated else "0"])
        attitude_rows.append(attitude_row)
        attitude_margins.append(
            float(np.min(attitude_rho - np.abs(attitude_terms.qv)))
        )
        attitude_errors.append(attitude_terms.qv.copy())
        attitude_true_disturbances.append(attitude_disturbance.copy())
        attitude_estimates.append(attitude_estimate.copy())
        attitude_torques.append(attitude_control.torque_frd_n_m.copy())
        orthogonality_errors.append(
            float(np.max(np.abs(rotation.T @ rotation - np.eye(3))))
        )
        determinant_errors.append(abs(float(np.linalg.det(rotation)) - 1.0))

        if index + 1 < SAMPLE_COUNT:
            held_velocity = velocity.copy()
            held_position_input = position_input.copy()
            acceleration = position_input + position_disturbance
            position = position + dt * velocity + 0.5 * dt**2 * acceleration
            velocity = velocity + dt * acceleration

            held_angular_velocity = angular_velocity.copy()
            held_attitude_input = attitude_input.copy()
            next_angular_velocity = _attitude_rk4_step(
                angular_velocity,
                attitude_control.torque_frd_n_m,
                attitude_disturbance,
                fixture.inertia_kg_m2,
                dt,
            )
            midpoint_angular_velocity = 0.5 * (
                angular_velocity + next_angular_velocity
            )
            rotation = rotation @ _so3_exponential(
                dt * midpoint_angular_velocity
            )
            angular_velocity = next_angular_velocity

    position_error_array = np.asarray(position_errors)
    position_true_array = np.asarray(position_true_disturbances)
    position_estimate_array = np.asarray(position_estimates)
    attitude_error_array = np.asarray(attitude_errors)
    attitude_true_array = np.asarray(attitude_true_disturbances)
    attitude_estimate_array = np.asarray(attitude_estimates)
    metrics: Dict[str, object] = {
        "position": {
            "minimum_envelope_margin_m": min(position_margins),
            "final_max_abs_error_m": float(
                np.max(np.abs(position_error_array[-1]))
            ),
            "disturbance_rmse_m_s2": np.sqrt(
                np.mean((position_estimate_array - position_true_array) ** 2, axis=0)
            ).tolist(),
            "maximum_abs_force_n": float(
                np.max(np.abs(np.asarray(position_forces)))
            ),
        },
        "attitude": {
            "minimum_envelope_margin": min(attitude_margins),
            "final_max_abs_qv": float(np.max(np.abs(attitude_error_array[-1]))),
            "disturbance_rmse_rad_s2": np.sqrt(
                np.mean((attitude_estimate_array - attitude_true_array) ** 2, axis=0)
            ).tolist(),
            "maximum_abs_torque_n_m": float(
                np.max(np.abs(np.asarray(attitude_torques)))
            ),
            "maximum_orthogonality_error": max(orthogonality_errors),
            "maximum_determinant_error": max(determinant_errors),
        },
    }
    return ClosedLoopData(position_rows, attitude_rows, metrics, paper, fixture)


def _write_csv(path: Path, columns: Sequence[str], rows: Sequence[Sequence[str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(columns)
        writer.writerows(rows)


def write_closed_loop_vectors(output_dir: Path) -> Tuple[Path, Path, Path]:
    """写出两份闭环 CSV 和共同清单。"""

    data = generate_closed_loop_data()
    output_dir.mkdir(parents=True, exist_ok=True)
    position_path = output_dir / "closed_loop_position_v1.csv"
    attitude_path = output_dir / "closed_loop_attitude_v1.csv"
    manifest_path = output_dir / "closed_loop_v1_manifest.json"
    _write_csv(position_path, POSITION_COLUMNS, data.position_rows)
    _write_csv(attitude_path, ATTITUDE_COLUMNS, data.attitude_rows)
    source_paths = {
        "generator": Path(__file__),
        "equations": REPO_ROOT / "tools/pregme_reference/equations.py",
        "envelope": REPO_ROOT / "tools/pregme_reference/envelope.py",
        "eso": REPO_ROOT / "tools/pregme_reference/eso.py",
        "attitude": REPO_ROOT / "tools/pregme_reference/attitude.py",
        "control": REPO_ROOT / "tools/pregme_reference/control.py",
    }
    manifest = {
        "schema_version": 1,
        "dataset_id": SCENARIO_ID,
        "sample_count": SAMPLE_COUNT,
        "dt_s": float(data.fixture.expected_attitude_period_s),
        "duration_s": float((SAMPLE_COUNT - 1) * data.fixture.expected_attitude_period_s),
        "causal_order": "advance ESO with previous y/u; estimate at current y; compute current control; advance truth",
        "truth_integration": {
            "position": "exact constant-acceleration ZOH update",
            "angular_velocity": "RK4 with held torque and disturbance",
            "rotation": "SO(3) exponential at interval midpoint angular velocity (A-07)",
        },
        "desired_attitude": "constant identity; omega_d=omega_d_dot=0",
        "disturbance": {
            "position": "0 before 2 s; constant [0.35,-0.25,0.18] to 8 s; then 0.5 Hz three-axis sine about that base",
            "attitude": "0 before 2 s; constant [0.18,-0.12,0.10] to 8 s; then 0.5 Hz three-axis sine about that base",
            "source": "deterministic synthetic coupling proxy; not paper experiment data",
        },
        "position": {
            "columns": POSITION_COLUMNS,
            "csv_sha256": _sha256(position_path),
            "frame": "NED",
            "initial_position_m": [0.0, 0.0, -1.5],
            "desired_position_m": [0.8, -0.8, -0.7],
        },
        "attitude": {
            "columns": ATTITUDE_COLUMNS,
            "csv_sha256": _sha256(attitude_path),
            "frame": "FRD body with rotations FRD-to-NED",
            "initial_rotation": "0.3 rad about FRD x axis",
            "inertia_kg_m2": data.fixture.inertia_kg_m2.tolist(),
            "inertia_status": data.fixture.inertia_status,
            "inertia_source": data.fixture.inertia_source,
            "inertia_replacement_condition": data.fixture.inertia_replacement_condition,
            "inertia_failure_consequence": data.fixture.inertia_failure_consequence,
        },
        "metrics": data.metrics,
        "paper": {
            "identifier": data.paper.paper.identifier,
            "source_sha256": data.paper.paper.source_sha256,
            "paper_default_sha256": paper_default_sha256(),
        },
        "fixture": {
            "id": data.fixture.fixture_id,
            "parameter_set_sha256": data.fixture.parameter_set_sha256,
            "watermark": data.fixture.watermark,
            "file_sha256": data.fixture.file_sha256,
            "metadata_sha256": data.fixture.metadata_sha256,
        },
        "implementation_baseline_commit": IMPLEMENTATION_BASELINE_COMMIT,
        "source_sha256": {name: _sha256(path) for name, path in source_paths.items()},
        "python": "3.10.12",
        "numpy": np.__version__,
        "randomness": "none",
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return position_path, attitude_path, manifest_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=REPO_ROOT / "test/pregme/data"
    )
    args = parser.parse_args()
    position_path, attitude_path, manifest_path = write_closed_loop_vectors(
        args.output_dir
    )
    print(f"position CSV: {position_path}")
    print(f"attitude CSV: {attitude_path}")
    print(f"manifest: {manifest_path}")
    print(f"samples: {SAMPLE_COUNT}")
    print(f"baseline: {IMPLEMENTATION_BASELINE_COMMIT}")


if __name__ == "__main__":
    main()
