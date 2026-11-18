"""M2 T06 统一复现图的格式、水印、哈希与确定性测试。"""

import hashlib
import json
from pathlib import Path
import struct

from tools.pregme_reference.generate_closed_loop_vectors import write_closed_loop_vectors
from tools.pregme_reference.plot_reproduction import FIGURE_STEMS
from tools.pregme_reference.plot_reproduction import WATERMARK
from tools.pregme_reference.plot_reproduction import generate_reproduction_plots


REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "test/pregme/data"
FIGURE_DIR = REPO_ROOT / "docs/figures/m2"
MANIFEST_PATH = FIGURE_DIR / "m2_plot_manifest.json"


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _png_dimensions(path):
    header = path.read_bytes()[:24]
    assert header[:8] == b"\x89PNG\r\n\x1a\n"
    assert header[12:16] == b"IHDR"
    return struct.unpack(">II", header[16:24])


def test_m2_fig_01_all_png_and_svg_assets_exist_with_fixed_canvas():
    for stem in FIGURE_STEMS:
        png = FIGURE_DIR / f"{stem}.png"
        svg = FIGURE_DIR / f"{stem}.svg"
        assert png.is_file() and svg.is_file()
        assert _png_dimensions(png) == (1800, 900)
        svg_text = svg.read_text(encoding="utf-8")
        assert "viewBox=\"0 0 864 432\"" in svg_text


def test_m2_fig_02_every_figure_carries_both_explicit_watermarks():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    fixture_watermark = "FIXTURE:m2_fixture_v1:072d67e9ccfb"
    assert manifest["watermarks"] == [WATERMARK, fixture_watermark]
    for stem in FIGURE_STEMS:
        svg_text = (FIGURE_DIR / f"{stem}.svg").read_text(encoding="utf-8")
        assert WATERMARK in svg_text
        assert fixture_watermark in svg_text


def test_m2_fig_03_manifest_hashes_all_sources_data_and_assets():
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    expected_sources = {
        "plot_reproduction": REPO_ROOT / "tools/pregme_reference/plot_reproduction.py",
        "closed_loop_manifest": DATA_DIR / "closed_loop_v1_manifest.json",
        "closed_loop_position": DATA_DIR / "closed_loop_position_v1.csv",
        "closed_loop_attitude": DATA_DIR / "closed_loop_attitude_v1.csv",
    }
    for name, path in expected_sources.items():
        assert manifest["source_sha256"][name] == _sha256(path)
    assert set(manifest["assets_sha256"]) == {
        f"{stem}.{extension}"
        for stem in FIGURE_STEMS
        for extension in ("png", "svg")
    }
    for filename, expected_hash in manifest["assets_sha256"].items():
        assert _sha256(FIGURE_DIR / filename) == expected_hash
    assert manifest["backend"].lower() == "agg"
    assert manifest["pixel_size"] == [1800, 900]


def test_m2_fig_04_plot_regeneration_is_byte_identical(tmp_path):
    output_dir = tmp_path / "figures"
    generated_manifest = generate_reproduction_plots(DATA_DIR, output_dir)
    assert generated_manifest.read_bytes() == MANIFEST_PATH.read_bytes()
    for stem in FIGURE_STEMS:
        for extension in ("png", "svg"):
            assert (output_dir / f"{stem}.{extension}").read_bytes() == (
                FIGURE_DIR / f"{stem}.{extension}"
            ).read_bytes()


def test_m2_fig_05_full_data_and_plot_pipeline_is_byte_identical(tmp_path):
    generated_data_dir = tmp_path / "data"
    generated_figure_dir = tmp_path / "figures"
    generated_position, generated_attitude, generated_data_manifest = write_closed_loop_vectors(generated_data_dir)
    generated_plot_manifest = generate_reproduction_plots(generated_data_dir, generated_figure_dir)
    assert generated_position.read_bytes() == (DATA_DIR / "closed_loop_position_v1.csv").read_bytes()
    assert generated_attitude.read_bytes() == (DATA_DIR / "closed_loop_attitude_v1.csv").read_bytes()
    assert generated_data_manifest.read_bytes() == (DATA_DIR / "closed_loop_v1_manifest.json").read_bytes()
    assert generated_plot_manifest.read_bytes() == MANIFEST_PATH.read_bytes()
