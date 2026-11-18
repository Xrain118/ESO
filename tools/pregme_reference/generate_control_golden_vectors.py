"""生成 M2 PR4 位置/姿态控制律规范算例黄金向量。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

from .control import attitude_control_torque
from .control import attitude_error_terms
from .control import position_control_config_from_paper
from .control import position_control_force
from .control import position_error_terms
from .control import position_known_acceleration
from .equations import load_paper_default
from .equations import paper_default_sha256
from .generate_attitude_golden_vectors import load_attitude_fixture_context


REPO_ROOT = Path(__file__).resolve().parents[2]
IMPLEMENTATION_BASELINE_COMMIT = "a1450bc177ad7316a6a3251261380b511abd5f62"


def _axis_columns(prefix: str) -> List[str]:
    return [f"{prefix}_{axis}" for axis in ("x", "y", "z")]


def _matrix_columns(prefix: str) -> List[str]:
    return [f"{prefix}_r{row}c{column}" for row in range(3) for column in range(3)]


POSITION_CSV_COLUMNS = (
    ["case_id"]
    + _axis_columns("position_ned_m")
    + _axis_columns("velocity_ned_m_s")
    + _axis_columns("desired_position_ned_m")
    + _axis_columns("desired_velocity_ned_m_s")
    + _axis_columns("desired_acceleration_ned_m_s2")
    + _axis_columns("beta_ned_m")
    + _axis_columns("beta_dot_ned_m_s")
    + _axis_columns("beta_ddot_ned_m_s2")
    + _axis_columns("disturbance_hat_ned_m_s2")
    + _axis_columns("tracking_error_ned_m")
    + _axis_columns("z_ned_m")
    + _axis_columns("z_dot_ned_m_s")
    + _axis_columns("sliding_ned_m_s")
    + _axis_columns("force_ned_n")
    + _axis_columns("known_acceleration_ned_m_s2")
)

ATTITUDE_CSV_COLUMNS = (
    ["case_id"]
    + _matrix_columns("rotation_ned_from_frd")
    + _matrix_columns("desired_rotation_ned_from_frd")
    + _axis_columns("angular_velocity_frd_rad_s")
    + _axis_columns("desired_angular_velocity_rad_s")
    + _axis_columns("desired_angular_acceleration_rad_s2")
    + _axis_columns("beta")
    + _axis_columns("beta_dot_s_inv")
    + _axis_columns("beta_ddot_s_inv2")
    + _axis_columns("disturbance_hat_frd_rad_s2")
    + ["q0"]
    + _axis_columns("qv")
    + ["q0_dot"]
    + _axis_columns("qv_dot")
    + _matrix_columns("q_matrix")
    + _matrix_columns("q_matrix_dot")
    + _axis_columns("omega_tilde_rad_s")
    + _axis_columns("z")
    + _axis_columns("z_dot_s_inv")
    + _axis_columns("sliding_s_inv")
    + _axis_columns("nonlinear_term_frd_rad_s2")
    + _axis_columns("torque_frd_n_m")
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _float_text(value: float) -> str:
    return format(np.float64(value), ".17g")


def _rotation(axis: Sequence[float], angle: float) -> np.ndarray:
    unit_axis = np.asarray(axis, dtype=np.float64)
    unit_axis = unit_axis / np.linalg.norm(unit_axis)
    x, y, z = unit_axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3)
        + np.sin(angle) * skew
        + (1.0 - np.cos(angle)) * (skew @ skew)
    )


def _position_cases() -> Iterable[Tuple[str, Dict[str, np.ndarray]]]:
    zero = np.zeros(3, dtype=np.float64)
    yield "hover_zero_error", {
        "position": zero,
        "velocity": zero,
        "desired_position": zero,
        "desired_velocity": zero,
        "desired_acceleration": zero,
        "beta": zero,
        "beta_dot": zero,
        "beta_ddot": zero,
        "disturbance_hat": zero,
    }
    yield "general_tracking", {
        "position": np.array([0.3, -0.4, -1.0]),
        "velocity": np.array([0.1, -0.05, 0.02]),
        "desired_position": np.array([0.8, -0.8, -0.7]),
        "desired_velocity": np.array([0.02, 0.01, -0.03]),
        "desired_acceleration": np.array([0.2, -0.1, 0.05]),
        "beta": np.array([-0.4, 0.3, -0.2]),
        "beta_dot": np.array([0.05, -0.02, 0.01]),
        "beta_ddot": np.array([-0.1, 0.08, -0.04]),
        "disturbance_hat": np.array([0.3, -0.2, 0.1]),
    }
    yield "disturbance_compensation", {
        "position": np.array([0.2, -0.1, -0.8]),
        "velocity": np.array([0.05, 0.02, -0.01]),
        "desired_position": np.array([0.2, -0.1, -0.8]),
        "desired_velocity": np.array([0.05, 0.02, -0.01]),
        "desired_acceleration": np.array([0.1, 0.2, -0.1]),
        "beta": zero,
        "beta_dot": zero,
        "beta_ddot": zero,
        "disturbance_hat": np.array([0.4, -0.3, 0.2]),
    }


def _attitude_cases() -> Iterable[Tuple[str, Dict[str, np.ndarray]]]:
    zero = np.zeros(3, dtype=np.float64)
    identity = np.eye(3, dtype=np.float64)
    yield "identity_stationary", {
        "rotation": identity,
        "desired_rotation": identity,
        "omega": zero,
        "omega_d": zero,
        "omega_d_dot": zero,
        "beta": zero,
        "beta_dot": zero,
        "beta_ddot": zero,
        "disturbance_hat": zero,
    }
    yield "single_axis_error", {
        "rotation": _rotation([1.0, 0.0, 0.0], 0.3),
        "desired_rotation": identity,
        "omega": np.array([0.08, 0.0, 0.0]),
        "omega_d": zero,
        "omega_d_dot": zero,
        "beta": np.array([0.05, 0.0, 0.0]),
        "beta_dot": np.array([-0.02, 0.0, 0.0]),
        "beta_ddot": np.array([0.01, 0.0, 0.0]),
        "disturbance_hat": np.array([0.2, 0.0, 0.0]),
    }
    yield "coupled_motion", {
        "rotation": _rotation([1.0, 2.0, -1.0], 0.4),
        "desired_rotation": _rotation([0.0, 0.0, 1.0], 0.2),
        "omega": np.array([0.3, -0.2, 0.15]),
        "omega_d": np.array([0.04, -0.03, 0.08]),
        "omega_d_dot": np.array([0.01, 0.02, -0.015]),
        "beta": np.array([0.08, -0.04, 0.03]),
        "beta_dot": np.array([-0.03, 0.02, -0.01]),
        "beta_ddot": np.array([0.02, -0.015, 0.01]),
        "disturbance_hat": np.array([0.12, -0.08, 0.05]),
    }


def generate_rows() -> Tuple[List[List[str]], List[List[str]], Dict[str, object]]:
    """从冻结 YAML 和 fixture 生成位置/姿态规范算例的 CSV 行。"""

    paper = load_paper_default()
    fixture = load_attitude_fixture_context()
    position_config = position_control_config_from_paper(paper)
    position_rows: List[List[str]] = []
    for case_id, values in _position_cases():
        errors = position_error_terms(
            values["position"],
            values["velocity"],
            values["desired_position"],
            values["desired_velocity"],
            values["beta"],
            values["beta_dot"],
            position_config.lambda_,
        )
        force = position_control_force(
            position_config,
            errors,
            values["desired_acceleration"],
            values["beta_ddot"],
            values["disturbance_hat"],
        )
        known_acceleration = position_known_acceleration(
            force, position_config.total_mass_kg
        )
        row = [case_id]
        for vector in (
            values["position"],
            values["velocity"],
            values["desired_position"],
            values["desired_velocity"],
            values["desired_acceleration"],
            values["beta"],
            values["beta_dot"],
            values["beta_ddot"],
            values["disturbance_hat"],
            errors.tracking_error_ned_m,
            errors.z_ned_m,
            errors.z_dot_ned_m_s,
            errors.sliding_ned_m_s,
            force,
            known_acceleration,
        ):
            row.extend(_float_text(value) for value in vector)
        position_rows.append(row)

    attitude_rows: List[List[str]] = []
    for case_id, values in _attitude_cases():
        errors = attitude_error_terms(
            values["rotation"],
            values["desired_rotation"],
            values["omega"],
            values["omega_d"],
            values["beta"],
            values["beta_dot"],
            paper.attitude.control.lambda_diagonal,
        )
        control = attitude_control_torque(
            fixture.inertia_kg_m2,
            values["omega"],
            errors,
            values["omega_d"],
            values["omega_d_dot"],
            values["beta_ddot"],
            values["disturbance_hat"],
            paper.attitude.control.lambda_diagonal,
            paper.attitude.control.k_diagonal,
        )
        row = [case_id]
        for matrix in (values["rotation"], values["desired_rotation"]):
            row.extend(_float_text(value) for value in matrix.reshape(-1))
        for vector in (
            values["omega"],
            values["omega_d"],
            values["omega_d_dot"],
            values["beta"],
            values["beta_dot"],
            values["beta_ddot"],
            values["disturbance_hat"],
        ):
            row.extend(_float_text(value) for value in vector)
        row.append(_float_text(errors.q0))
        row.extend(_float_text(value) for value in errors.qv)
        row.append(_float_text(errors.q0_dot))
        row.extend(_float_text(value) for value in errors.qv_dot)
        for matrix in (errors.q_matrix, errors.q_matrix_dot):
            row.extend(_float_text(value) for value in matrix.reshape(-1))
        for vector in (
            errors.omega_tilde,
            errors.z,
            errors.z_dot,
            errors.sliding,
            control.nonlinear_term_frd_rad_s2,
            control.torque_frd_n_m,
        ):
            row.extend(_float_text(value) for value in vector)
        attitude_rows.append(row)

    context = {
        "paper": paper,
        "fixture": fixture,
        "position_config": position_config,
    }
    return position_rows, attitude_rows, context


def _write_csv(path: Path, columns: Sequence[str], rows: Iterable[Sequence[str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(columns)
        writer.writerows(rows)


def write_golden_vectors(output_dir: Path) -> Tuple[Path, Path, Path]:
    """写出确定性位置/姿态 CSV 和共同 JSON 清单。"""

    position_rows, attitude_rows, context = generate_rows()
    output_dir.mkdir(parents=True, exist_ok=True)
    position_path = output_dir / "position_control_v1.csv"
    attitude_path = output_dir / "attitude_control_v1.csv"
    manifest_path = output_dir / "control_v1_manifest.json"
    _write_csv(position_path, POSITION_CSV_COLUMNS, position_rows)
    _write_csv(attitude_path, ATTITUDE_CSV_COLUMNS, attitude_rows)

    paper = context["paper"]
    fixture = context["fixture"]
    position_config = context["position_config"]
    source_paths = {
        "control": REPO_ROOT / "tools/pregme_reference/control.py",
        "generator": Path(__file__),
        "envelope": REPO_ROOT / "tools/pregme_reference/envelope.py",
        "attitude": REPO_ROOT / "tools/pregme_reference/attitude.py",
    }
    manifest: Dict[str, object] = {
        "schema_version": 1,
        "dataset_id": "pregme_control_reference_v1",
        "formula": ["(6)", "(8)", "(9)", "(15)", "(16)", "(17)", "(18)", "(19)", "(20)", "(21)"],
        "frames": {"position": "NED", "attitude": "FRD body with rotations FRD-to-NED"},
        "desired_attitude_derivatives": "omega_d and omega_d_dot are explicit caller inputs",
        "saturation": "disabled",
        "position": {
            "columns": POSITION_CSV_COLUMNS,
            "case_count": len(position_rows),
            "csv_sha256": _sha256(position_path),
            "total_mass_kg": float(position_config.total_mass_kg),
            "k": position_config.k.tolist(),
            "lambda": position_config.lambda_.tolist(),
        },
        "attitude": {
            "columns": ATTITUDE_CSV_COLUMNS,
            "case_count": len(attitude_rows),
            "csv_sha256": _sha256(attitude_path),
            "k": paper.attitude.control.k_diagonal.tolist(),
            "lambda": paper.attitude.control.lambda_diagonal.tolist(),
            "inertia_kg_m2": fixture.inertia_kg_m2.tolist(),
            "inertia_status": fixture.inertia_status,
            "inertia_source": fixture.inertia_source,
            "inertia_replacement_condition": fixture.inertia_replacement_condition,
            "inertia_failure_consequence": fixture.inertia_failure_consequence,
        },
        "units": {
            "position": "m,m/s,m/s^2,N",
            "attitude": "rad/s,rad/s^2,N*m,kg*m^2",
        },
        "paper": {
            "identifier": paper.paper.identifier,
            "source_sha256": paper.paper.source_sha256,
            "paper_default_sha256": paper_default_sha256(),
        },
        "fixture": {
            "id": fixture.fixture_id,
            "parameter_set_sha256": fixture.parameter_set_sha256,
            "watermark": fixture.watermark,
            "file_sha256": fixture.file_sha256,
            "metadata_sha256": fixture.metadata_sha256,
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
    position_path, attitude_path, manifest_path = write_golden_vectors(args.output_dir)
    print(f"position CSV: {position_path}")
    print(f"attitude CSV: {attitude_path}")
    print(f"manifest: {manifest_path}")
    print(f"baseline: {IMPLEMENTATION_BASELINE_COMMIT}")


if __name__ == "__main__":
    main()
