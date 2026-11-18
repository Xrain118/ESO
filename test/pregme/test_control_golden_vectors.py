"""M2 PR4 控制律黄金 CSV/JSON 结构、哈希与再生测试。"""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from tools.pregme_reference.generate_control_golden_vectors import (
    ATTITUDE_CSV_COLUMNS,
)
from tools.pregme_reference.generate_control_golden_vectors import (
    IMPLEMENTATION_BASELINE_COMMIT,
)
from tools.pregme_reference.generate_control_golden_vectors import (
    POSITION_CSV_COLUMNS,
)
from tools.pregme_reference.generate_control_golden_vectors import write_golden_vectors


REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "test/pregme/data"
POSITION_PATH = DATA_DIR / "position_control_v1.csv"
ATTITUDE_PATH = DATA_DIR / "attitude_control_v1.csv"
MANIFEST_PATH = DATA_DIR / "control_v1_manifest.json"


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_csv(path):
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return reader.fieldnames, list(reader)


def test_m2_gold_ctrl_01_csv_schema_units_cases_and_finite_values():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    position_columns, position_rows = _read_csv(POSITION_PATH)
    attitude_columns, attitude_rows = _read_csv(ATTITUDE_PATH)
    assert position_columns == POSITION_CSV_COLUMNS
    assert attitude_columns == ATTITUDE_CSV_COLUMNS
    assert manifest["position"]["columns"] == POSITION_CSV_COLUMNS
    assert manifest["attitude"]["columns"] == ATTITUDE_CSV_COLUMNS
    assert len(position_rows) == manifest["position"]["case_count"] == 3
    assert len(attitude_rows) == manifest["attitude"]["case_count"] == 3
    assert [row["case_id"] for row in position_rows] == [
        "hover_zero_error",
        "general_tracking",
        "disturbance_compensation",
    ]
    assert [row["case_id"] for row in attitude_rows] == [
        "identity_stationary",
        "single_axis_error",
        "coupled_motion",
    ]
    for rows, columns in (
        (position_rows, POSITION_CSV_COLUMNS),
        (attitude_rows, ATTITUDE_CSV_COLUMNS),
    ):
        numeric = np.asarray(
            [[float(row[column]) for column in columns[1:]] for row in rows]
        )
        assert np.all(np.isfinite(numeric))
    assert manifest["frames"]["position"] == "NED"
    assert "FRD" in manifest["frames"]["attitude"]
    assert manifest["saturation"] == "disabled"


def test_m2_gold_ctrl_02_all_hashes_fixture_and_placeholder_match():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["position"]["csv_sha256"] == _sha256(POSITION_PATH)
    assert manifest["attitude"]["csv_sha256"] == _sha256(ATTITUDE_PATH)
    for name, relative in (
        ("control", "tools/pregme_reference/control.py"),
        ("generator", "tools/pregme_reference/generate_control_golden_vectors.py"),
        ("envelope", "tools/pregme_reference/envelope.py"),
        ("attitude", "tools/pregme_reference/attitude.py"),
    ):
        assert manifest["source_sha256"][name] == _sha256(REPO_ROOT / relative)
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
    assert manifest["attitude"]["inertia_status"] == "placeholder"
    assert "HIL" in manifest["attitude"]["inertia_replacement_condition"]
    assert manifest["implementation_baseline_commit"] == IMPLEMENTATION_BASELINE_COMMIT
    assert manifest["randomness"] == "none"


def test_m2_gold_ctrl_03_regeneration_is_byte_identical(tmp_path):
    position, attitude, manifest = write_golden_vectors(tmp_path)
    assert position.read_bytes() == POSITION_PATH.read_bytes()
    assert attitude.read_bytes() == ATTITUDE_PATH.read_bytes()
    assert manifest.read_bytes() == MANIFEST_PATH.read_bytes()


def test_m2_gold_ctrl_04_prior_eso_golden_vectors_remain_unchanged():
    assert _sha256(DATA_DIR / "position_eso_v1.csv") == (
        "04de6cc179b16698d32ef10338aff0855afbb903a482e648b2aa8c7e51916d14"
    )
    assert _sha256(DATA_DIR / "attitude_eso_v1.csv") == (
        "2b1bd64596c8e73488502b89ca32485a1daa20ba088992119fb5fd7f2099e9ac"
    )
