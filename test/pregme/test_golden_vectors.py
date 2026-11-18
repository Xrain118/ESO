"""M2 位置 ESO 黄金 CSV/JSON 的结构、锚点与再生测试。"""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.pregme_reference.generate_golden_vectors import CSV_COLUMNS
from tools.pregme_reference.generate_golden_vectors import DEFAULT_SEED
from tools.pregme_reference.generate_golden_vectors import IMPLEMENTATION_BASELINE_COMMIT
from tools.pregme_reference.generate_golden_vectors import generate_position_scenario
from tools.pregme_reference.generate_golden_vectors import write_scenario


REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "test/pregme/data"
CSV_PATH = DATA_DIR / "position_eso_v1.csv"
MANIFEST_PATH = DATA_DIR / "position_eso_v1_manifest.json"
GENERATOR_PATH = REPO_ROOT / "tools/pregme_reference/generate_golden_vectors.py"


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_rows():
    with CSV_PATH.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        return reader.fieldnames, rows


def test_m2_gold_pos_01_schema_timestamps_units_and_finite_values():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    fieldnames, rows = _read_rows()
    assert fieldnames == CSV_COLUMNS
    assert manifest["columns"] == CSV_COLUMNS
    assert manifest["sample_count"] == 1201
    assert len(rows) == manifest["sample_count"]
    assert manifest["frame"] == "NED"
    assert manifest["dt_s"] == 0.01
    assert manifest["units"]["velocity_ned_m_s"] == "m/s"
    assert manifest["units"]["disturbance_hat_ned_m_s2"] == "m/s^2"

    timestamps = np.asarray([int(row["timestamp_us"]) for row in rows], dtype=np.int64)
    time_s = np.asarray([float(row["time_s"]) for row in rows], dtype=np.float64)
    np.testing.assert_array_equal(np.diff(timestamps), np.full(1200, 10_000))
    np.testing.assert_allclose(np.diff(time_s), 0.01, rtol=0.0, atol=2.0e-15)

    numeric_columns = CSV_COLUMNS[1:-2]
    numeric = np.asarray(
        [[float(row[column]) for column in numeric_columns] for row in rows],
        dtype=np.float64,
    )
    assert np.all(np.isfinite(numeric))
    assert rows[0]["status"] == "initialized"
    assert rows[0]["integrated"] == "0"
    assert all(row["status"] == "integrated" for row in rows[1:])
    assert all(row["integrated"] == "1" for row in rows[1:])


def test_m2_gold_pos_02_all_traceability_hashes_match():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["csv_sha256"] == _sha256(CSV_PATH)
    assert manifest["generator_source_sha256"] == _sha256(GENERATOR_PATH)
    assert manifest["paper"]["paper_default_sha256"] == _sha256(
        REPO_ROOT / "config/paper_default.yaml"
    )
    assert manifest["paper"]["source_sha256"] == (
        "790069c4f56680fa0b937d67854beaff2057f74de847f159d3a825d2b63aa4a9"
    )
    assert manifest["fixture"]["file_sha256"] == _sha256(
        REPO_ROOT / "config/m2_fixture.yaml"
    )
    assert manifest["fixture"]["parameter_set_sha256"] == (
        "072d67e9ccfbfb78abbf7fed9625f4490b8a76a259a041abe97d864a17036764"
    )
    assert manifest["fixture"]["watermark"] == "FIXTURE:m2_fixture_v1:072d67e9ccfb"
    assert manifest["implementation_baseline_commit"] == IMPLEMENTATION_BASELINE_COMMIT
    assert manifest["random_noise_used"] is False
    assert manifest["seed"] == DEFAULT_SEED


def test_m2_gold_pos_03_regeneration_is_byte_identical(tmp_path):
    regenerated = generate_position_scenario("composite", seed=DEFAULT_SEED)
    regenerated_csv, regenerated_manifest = write_scenario(regenerated, tmp_path)
    assert regenerated_csv.read_bytes() == CSV_PATH.read_bytes()
    assert regenerated_manifest.read_bytes() == MANIFEST_PATH.read_bytes()
