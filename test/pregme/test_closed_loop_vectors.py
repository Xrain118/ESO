"""M2 T06 合成闭环时序、语义指标、哈希和再生测试。"""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.pregme_reference.equations import load_paper_default
from tools.pregme_reference.eso import variable_gain_eso_estimate
from tools.pregme_reference.eso import variable_gain_eso_rk4_step
from tools.pregme_reference.generate_closed_loop_vectors import ATTITUDE_COLUMNS
from tools.pregme_reference.generate_closed_loop_vectors import POSITION_COLUMNS
from tools.pregme_reference.generate_closed_loop_vectors import SAMPLE_COUNT
from tools.pregme_reference.generate_closed_loop_vectors import write_closed_loop_vectors


REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "test/pregme/data"
POSITION_PATH = DATA_DIR / "closed_loop_position_v1.csv"
ATTITUDE_PATH = DATA_DIR / "closed_loop_attitude_v1.csv"
MANIFEST_PATH = DATA_DIR / "closed_loop_v1_manifest.json"
AXES = ("x", "y", "z")


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_csv(path):
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        columns = list(reader.fieldnames or [])
    numeric = {
        column: np.asarray([float(row[column]) for row in rows], dtype=np.float64)
        for column in columns
        if column not in {"status", "integrated"}
    }
    return columns, rows, numeric


def _vector(data, prefix):
    return np.column_stack([data[f"{prefix}_{axis}"] for axis in AXES])


def test_m2_close_01_schema_time_status_and_finite_values():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    position_columns, position_rows, position = _read_csv(POSITION_PATH)
    attitude_columns, attitude_rows, attitude = _read_csv(ATTITUDE_PATH)
    assert position_columns == POSITION_COLUMNS == manifest["position"]["columns"]
    assert attitude_columns == ATTITUDE_COLUMNS == manifest["attitude"]["columns"]
    assert len(position_rows) == len(attitude_rows) == SAMPLE_COUNT == 1601
    for rows, data in ((position_rows, position), (attitude_rows, attitude)):
        assert rows[0]["status"] == "initialized"
        assert rows[0]["integrated"] == "0"
        assert all(row["status"] == "integrated" for row in rows[1:])
        assert all(row["integrated"] == "1" for row in rows[1:])
        assert np.array_equal(np.diff(data["timestamp_us"]), np.full(1600, 10000.0))
        assert np.allclose(data["time_s"], np.arange(SAMPLE_COUNT) * 0.01)
        assert all(np.all(np.isfinite(values)) for values in data.values())


def test_m2_close_02_first_sample_initializes_both_esos_exactly():
    _, _, position = _read_csv(POSITION_PATH)
    _, _, attitude = _read_csv(ATTITUDE_PATH)
    assert np.array_equal(_vector(position, "h_v_ned_m_s")[0], _vector(position, "velocity_ned_m_s")[0])
    assert np.array_equal(_vector(attitude, "h_omega_frd_rad_s")[0], _vector(attitude, "angular_velocity_frd_rad_s")[0])
    for data, prefixes in (
        (position, ("innovation_ned_m_s", "disturbance_hat_ned_m_s2")),
        (attitude, ("innovation_frd_rad_s", "disturbance_hat_frd_rad_s2")),
    ):
        for prefix in prefixes:
            assert np.array_equal(_vector(data, prefix)[0], np.zeros(3))


def test_m2_close_03_previous_sample_hold_and_current_innovation_are_reproducible():
    paper = load_paper_default()
    _, _, position = _read_csv(POSITION_PATH)
    _, _, attitude = _read_csv(ATTITUDE_PATH)
    cases = (
        (
            position,
            "h_v_ned_m_s",
            "velocity_ned_m_s",
            "known_acceleration_ned_m_s2",
            "innovation_ned_m_s",
            "disturbance_hat_ned_m_s2",
            paper.position.eso.alpha,
            paper.position.eso.epsilon,
        ),
        (
            attitude,
            "h_omega_frd_rad_s",
            "angular_velocity_frd_rad_s",
            "known_angular_acceleration_frd_rad_s2",
            "innovation_frd_rad_s",
            "disturbance_hat_frd_rad_s2",
            paper.attitude.eso.alpha,
            paper.attitude.eso.epsilon,
        ),
    )
    for data, h_name, y_name, u_name, innovation_name, estimate_name, alpha, epsilon in cases:
        h = _vector(data, h_name)
        y = _vector(data, y_name)
        u = _vector(data, u_name)
        innovation = _vector(data, innovation_name)
        estimate = _vector(data, estimate_name)
        for index in (1, 200, 800, 1600):
            expected_h = variable_gain_eso_rk4_step(
                h[index - 1], y[index - 1], u[index - 1], alpha, epsilon,
                paper.variable_gain.w, paper.variable_gain.d, 0.01,
            )
            expected_innovation, expected_estimate = variable_gain_eso_estimate(
                expected_h, y[index], alpha, epsilon,
                paper.variable_gain.w, paper.variable_gain.d,
            )
            assert np.array_equal(h[index], expected_h)
            assert np.array_equal(innovation[index], expected_innovation)
            assert np.array_equal(estimate[index], expected_estimate)


def test_m2_close_04_error_and_sliding_columns_match_frozen_definitions():
    paper = load_paper_default()
    _, _, position = _read_csv(POSITION_PATH)
    tracking_error = _vector(position, "position_ned_m") - _vector(position, "desired_position_ned_m")
    z = tracking_error - _vector(position, "beta_ned_m")
    z_dot = (
        _vector(position, "velocity_ned_m_s")
        - _vector(position, "desired_velocity_ned_m_s")
        - _vector(position, "beta_dot_ned_m_s")
    )
    sliding = z_dot + paper.position.control.lambda_diagonal * z
    assert np.array_equal(_vector(position, "tracking_error_ned_m"), tracking_error)
    assert np.array_equal(_vector(position, "z_ned_m"), z)
    assert np.array_equal(_vector(position, "z_dot_ned_m_s"), z_dot)
    assert np.array_equal(_vector(position, "sliding_ned_m_s"), sliding)

    _, _, attitude = _read_csv(ATTITUDE_PATH)
    attitude_z = _vector(attitude, "qv") - _vector(attitude, "beta")
    attitude_z_dot = _vector(attitude, "z_dot_s_inv")
    attitude_sliding = attitude_z_dot + paper.attitude.control.lambda_diagonal * attitude_z
    assert np.array_equal(_vector(attitude, "z"), attitude_z)
    assert np.array_equal(_vector(attitude, "sliding_s_inv"), attitude_sliding)


def test_m2_close_05_performance_envelopes_remain_strictly_satisfied():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    _, _, position = _read_csv(POSITION_PATH)
    _, _, attitude = _read_csv(ATTITUDE_PATH)
    position_margin = _vector(position, "rho_ned_m") - np.abs(_vector(position, "tracking_error_ned_m"))
    attitude_margin = _vector(attitude, "rho") - np.abs(_vector(attitude, "qv"))
    assert np.min(position_margin) > 0.02
    assert np.max(np.abs(_vector(position, "tracking_error_ned_m")[-1])) < 0.01
    assert np.min(attitude_margin) > 0.05
    assert np.max(np.abs(_vector(attitude, "qv")[-1])) < 3.0e-4
    assert manifest["metrics"]["position"]["minimum_envelope_margin_m"] == np.min(position_margin)
    assert manifest["metrics"]["attitude"]["minimum_envelope_margin"] == np.min(attitude_margin)


def test_m2_close_06_disturbance_schedule_and_phase_are_exact():
    _, _, position = _read_csv(POSITION_PATH)
    _, _, attitude = _read_csv(ATTITUDE_PATH)
    position_disturbance = _vector(position, "true_disturbance_ned_m_s2")
    attitude_disturbance = _vector(attitude, "true_disturbance_frd_rad_s2")
    assert np.array_equal(position_disturbance[199], np.zeros(3))
    assert np.array_equal(position_disturbance[200], np.array([0.35, -0.25, 0.18]))
    assert np.array_equal(attitude_disturbance[199], np.zeros(3))
    assert np.array_equal(attitude_disturbance[200], np.array([0.18, -0.12, 0.10]))
    phase = np.array([0.0, np.pi / 3.0, 2.0 * np.pi / 3.0])
    assert np.allclose(position_disturbance[800], np.array([0.35, -0.25, 0.18]) + np.array([0.12, 0.10, 0.08]) * np.sin(phase))
    assert np.allclose(attitude_disturbance[800], np.array([0.18, -0.12, 0.10]) + np.array([0.06, 0.05, 0.04]) * np.sin(phase))


def test_m2_close_07_rotation_control_and_metrics_are_finite_and_physical():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    _, _, position = _read_csv(POSITION_PATH)
    _, _, attitude = _read_csv(ATTITUDE_PATH)
    rotation_columns = [f"rotation_ned_from_frd_r{row}c{column}" for row in range(3) for column in range(3)]
    rotations = np.column_stack([attitude[column] for column in rotation_columns]).reshape(-1, 3, 3)
    orthogonality_error = np.max(np.abs(np.swapaxes(rotations, 1, 2) @ rotations - np.eye(3)))
    determinant_error = np.max(np.abs(np.linalg.det(rotations) - 1.0))
    assert orthogonality_error < 1.0e-10
    assert determinant_error < 1.0e-10
    assert np.all(np.isfinite(_vector(position, "force_ned_n")))
    assert np.all(np.isfinite(_vector(attitude, "torque_frd_n_m")))
    assert manifest["attitude"]["inertia_status"] == "placeholder"
    assert "not paper experiment data" in manifest["disturbance"]["source"]
    assert all(np.isfinite(value) for group in manifest["metrics"].values() for value in _flatten(group))


def _flatten(value):
    if isinstance(value, dict):
        for child in value.values():
            yield from _flatten(child)
    elif isinstance(value, list):
        for child in value:
            yield from _flatten(child)
    else:
        yield value


def test_m2_close_08_hashes_prior_vectors_and_regeneration_are_stable(tmp_path):
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["position"]["csv_sha256"] == _sha256(POSITION_PATH)
    assert manifest["attitude"]["csv_sha256"] == _sha256(ATTITUDE_PATH)
    for name, relative_path in {
        "generator": "tools/pregme_reference/generate_closed_loop_vectors.py",
        "equations": "tools/pregme_reference/equations.py",
        "envelope": "tools/pregme_reference/envelope.py",
        "eso": "tools/pregme_reference/eso.py",
        "attitude": "tools/pregme_reference/attitude.py",
        "control": "tools/pregme_reference/control.py",
    }.items():
        assert manifest["source_sha256"][name] == _sha256(REPO_ROOT / relative_path)
    prior_hashes = {
        "position_eso_v1.csv": "04de6cc179b16698d32ef10338aff0855afbb903a482e648b2aa8c7e51916d14",
        "attitude_eso_v1.csv": "2b1bd64596c8e73488502b89ca32485a1daa20ba088992119fb5fd7f2099e9ac",
        "position_control_v1.csv": "9b65638bef0180b28dacd707eff9b2b2bc1394c27c65a85bf1c5318a325579b7",
        "attitude_control_v1.csv": "fee1adc1165376eb3b8049a5b8b76e9ddc98cd8bf996c4840182a3e36d6e022b",
    }
    for filename, expected_hash in prior_hashes.items():
        assert _sha256(DATA_DIR / filename) == expected_hash
    generated_position, generated_attitude, generated_manifest = write_closed_loop_vectors(tmp_path)
    assert generated_position.read_bytes() == POSITION_PATH.read_bytes()
    assert generated_attitude.read_bytes() == ATTITUDE_PATH.read_bytes()
    assert generated_manifest.read_bytes() == MANIFEST_PATH.read_bytes()
