"""M2 姿态 ESO 黄金 CSV/JSON 的结构、锚点与再生测试。"""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.pregme_reference.generate_attitude_golden_vectors import CSV_COLUMNS
from tools.pregme_reference.generate_attitude_golden_vectors import DEFAULT_SEED
from tools.pregme_reference.generate_attitude_golden_vectors import (
    IMPLEMENTATION_BASELINE_COMMIT,
)
from tools.pregme_reference.generate_attitude_golden_vectors import (
    generate_attitude_scenario,
)
from tools.pregme_reference.generate_attitude_golden_vectors import write_scenario


REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "test/pregme/data"
CSV_PATH = DATA_DIR / "attitude_eso_v1.csv"
MANIFEST_PATH = DATA_DIR / "attitude_eso_v1_manifest.json"
POSITION_CSV_PATH = DATA_DIR / "position_eso_v1.csv"
POSITION_MANIFEST_PATH = DATA_DIR / "position_eso_v1_manifest.json"


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_rows():
    with CSV_PATH.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def test_m2_gold_att_01_schema_timestamps_units_and_finite_values():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    fieldnames, rows = _read_rows()
    assert fieldnames == CSV_COLUMNS
    assert manifest["columns"] == CSV_COLUMNS
    assert manifest["sample_count"] == 1201
    assert len(rows) == manifest["sample_count"]
    assert manifest["frame"] == "FRD"
    assert manifest["dt_s"] == 0.01
    assert manifest["units"]["angular_velocity_frd_rad_s"] == "rad/s"
    assert manifest["units"]["torque_frd_n_m"] == "N*m"
    assert manifest["units"]["disturbance_hat_frd_rad_s2"] == "rad/s^2"

    timestamps = np.asarray([int(row["timestamp_us"]) for row in rows])
    np.testing.assert_array_equal(np.diff(timestamps), np.full(1200, 10_000))
    numeric = np.asarray(
        [
            [float(row[column]) for column in CSV_COLUMNS[1:-2]]
            for row in rows
        ],
        dtype=np.float64,
    )
    assert np.all(np.isfinite(numeric))
    assert rows[0]["status"] == "initialized"
    assert rows[0]["integrated"] == "0"
    assert all(row["status"] == "integrated" for row in rows[1:])


def test_m2_gold_att_02_traceability_hashes_placeholder_and_watermark_match():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["csv_sha256"] == _sha256(CSV_PATH)
    assert manifest["source_sha256"]["attitude"] == _sha256(
        REPO_ROOT / "tools/pregme_reference/attitude.py"
    )
    assert manifest["source_sha256"]["shared_eso"] == _sha256(
        REPO_ROOT / "tools/pregme_reference/eso.py"
    )
    assert manifest["source_sha256"]["generator"] == _sha256(
        REPO_ROOT / "tools/pregme_reference/generate_attitude_golden_vectors.py"
    )
    assert manifest["paper"]["paper_default_sha256"] == _sha256(
        REPO_ROOT / "config/paper_default.yaml"
    )
    assert manifest["fixture"]["file_sha256"] == _sha256(
        REPO_ROOT / "config/m2_fixture.yaml"
    )
    assert manifest["fixture"]["metadata_sha256"] == _sha256(
        REPO_ROOT / "config/m2_fixture_metadata.yaml"
    )
    assert manifest["fixture"]["watermark"] == "FIXTURE:m2_fixture_v1:072d67e9ccfb"
    assert manifest["inertia"]["status"] == "placeholder"
    assert manifest["inertia"]["frame"] == "quadcopter_body_FRD"
    assert manifest["inertia"]["matrix_kg_m2"] == [
        [0.48, 0.0, 0.0],
        [0.0, 0.48, 0.0],
        [0.0, 0.0, 0.7],
    ]
    assert "HIL" in manifest["inertia"]["replacement_condition"]
    assert manifest["implementation_baseline_commit"] == IMPLEMENTATION_BASELINE_COMMIT
    assert manifest["seed"] == DEFAULT_SEED
    assert manifest["random_noise_used"] is False


def test_m2_gold_att_03_regeneration_is_byte_identical(tmp_path):
    regenerated = generate_attitude_scenario("composite", seed=DEFAULT_SEED)
    regenerated_csv, regenerated_manifest = write_scenario(regenerated, tmp_path)
    assert regenerated_csv.read_bytes() == CSV_PATH.read_bytes()
    assert regenerated_manifest.read_bytes() == MANIFEST_PATH.read_bytes()


def test_m2_gold_att_04_position_golden_vectors_remain_unchanged():
    assert _sha256(POSITION_CSV_PATH) == (
        "04de6cc179b16698d32ef10338aff0855afbb903a482e648b2aa8c7e51916d14"
    )
    manifest = json.loads(POSITION_MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["csv_sha256"] == _sha256(POSITION_CSV_PATH)
