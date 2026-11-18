"""绘制 M2 T06 统一合成闭环复现曲线并生成可审计清单。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


REPO_ROOT = Path(__file__).resolve().parents[2]
FIGURE_STEMS = (
    "m2_tracking_envelopes",
    "m2_eso_estimates",
    "m2_sliding_vectors",
    "m2_control_inputs",
)
AXES = ("x", "y", "z")
COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7")
WATERMARK = "SYNTHETIC M2 FIXTURE — NOT PAPER EXPERIMENT"
FIGSIZE_INCH = (12.0, 6.0)
DPI = 150


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _read_csv(path: Path) -> Tuple[List[str], Dict[str, np.ndarray]]:
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"CSV 缺少表头：{path}")
        rows = list(reader)
        columns = list(reader.fieldnames)
    if not rows:
        raise ValueError(f"CSV 不得为空：{path}")
    numeric: Dict[str, np.ndarray] = {}
    for column in columns:
        if column in ("status", "integrated"):
            continue
        try:
            numeric[column] = np.asarray(
                [float(row[column]) for row in rows], dtype=np.float64
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"CSV 数值列非法：{path.name}:{column}") from exc
        if not np.all(np.isfinite(numeric[column])):
            raise ValueError(f"CSV 数值列含非有限值：{path.name}:{column}")
    return columns, numeric


def _configure_style() -> None:
    matplotlib.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9.0,
            "axes.grid": True,
            "axes.axisbelow": True,
            "grid.alpha": 0.25,
            "grid.linewidth": 0.6,
            "lines.linewidth": 1.2,
            "path.simplify": False,
            "savefig.dpi": DPI,
            "svg.fonttype": "none",
            "svg.hashsalt": "eso-pregme-m2-v1",
        }
    )


def _new_figure(title: str) -> Tuple[plt.Figure, np.ndarray]:
    figure, axes = plt.subplots(2, 3, figsize=FIGSIZE_INCH, dpi=DPI, sharex=True)
    figure.suptitle(title, fontsize=12.0)
    figure.subplots_adjust(left=0.065, right=0.985, top=0.90, bottom=0.13, hspace=0.30)
    return figure, axes


def _add_watermark(figure: plt.Figure, fixture_watermark: str) -> None:
    figure.text(
        0.5,
        0.045,
        WATERMARK,
        color="#8B0000",
        fontsize=8.0,
        fontweight="bold",
        ha="center",
    )
    figure.text(
        0.5,
        0.018,
        fixture_watermark,
        color="#555555",
        fontsize=7.0,
        ha="center",
    )


def _finish_axes(axes: np.ndarray) -> None:
    for column, axis in enumerate(axes[1]):
        axis.set_xlabel("Time [s]")
    for row in axes:
        for axis in row:
            axis.axvline(2.0, color="#777777", linestyle=":", linewidth=0.8)
            axis.axvline(8.0, color="#777777", linestyle=":", linewidth=0.8)
    axes[0, 0].legend(loc="best", fontsize=7.0, framealpha=0.9)
    axes[1, 0].legend(loc="best", fontsize=7.0, framealpha=0.9)


def _save_figure(figure: plt.Figure, output_dir: Path, stem: str) -> List[Path]:
    png_path = output_dir / f"{stem}.png"
    svg_path = output_dir / f"{stem}.svg"
    figure.savefig(
        png_path,
        format="png",
        dpi=DPI,
        metadata={"Software": "ESO PreGME M2 reference"},
    )
    figure.savefig(
        svg_path,
        format="svg",
        dpi=DPI,
        metadata={"Creator": "ESO PreGME M2 reference", "Date": None},
    )
    plt.close(figure)
    svg_text = svg_path.read_text(encoding="utf-8")
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_text.splitlines()) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return [png_path, svg_path]


def _plot_tracking(
    position: Mapping[str, np.ndarray],
    attitude: Mapping[str, np.ndarray],
    fixture_watermark: str,
    output_dir: Path,
) -> List[Path]:
    figure, axes = _new_figure("M2 synthetic closed-loop tracking and envelopes")
    position_time = position["time_s"]
    attitude_time = attitude["time_s"]
    for index, axis_name in enumerate(AXES):
        top = axes[0, index]
        error = position[f"tracking_error_ned_m_{axis_name}"]
        beta = position[f"beta_ned_m_{axis_name}"]
        rho = position[f"rho_ned_m_{axis_name}"]
        top.plot(position_time, error, color=COLORS[0], label="tracking error")
        top.plot(position_time, beta, color=COLORS[1], linestyle="--", label="preset beta")
        top.plot(position_time, rho, color="#333333", linestyle=":", label="+/- envelope")
        top.plot(position_time, -rho, color="#333333", linestyle=":")
        top.set_title(f"Position {axis_name.upper()}")
        top.set_ylabel("Error [m]")

        bottom = axes[1, index]
        qv = attitude[f"qv_{axis_name}"]
        beta_q = attitude[f"beta_{axis_name}"]
        rho_q = attitude[f"rho_{axis_name}"]
        bottom.plot(attitude_time, qv, color=COLORS[0], label="quaternion vector")
        bottom.plot(attitude_time, beta_q, color=COLORS[1], linestyle="--", label="preset beta")
        bottom.plot(attitude_time, rho_q, color="#333333", linestyle=":", label="+/- envelope")
        bottom.plot(attitude_time, -rho_q, color="#333333", linestyle=":")
        bottom.set_title(f"Attitude {axis_name.upper()}")
        bottom.set_ylabel("Quaternion vector [-]")
    _finish_axes(axes)
    _add_watermark(figure, fixture_watermark)
    return _save_figure(figure, output_dir, "m2_tracking_envelopes")


def _plot_eso(
    position: Mapping[str, np.ndarray],
    attitude: Mapping[str, np.ndarray],
    fixture_watermark: str,
    output_dir: Path,
) -> List[Path]:
    figure, axes = _new_figure("M2 variable-gain ESO disturbance estimates")
    for index, axis_name in enumerate(AXES):
        top = axes[0, index]
        top.plot(
            position["time_s"],
            position[f"true_disturbance_ned_m_s2_{axis_name}"],
            color=COLORS[0],
            label="true synthetic disturbance",
        )
        top.plot(
            position["time_s"],
            position[f"disturbance_hat_ned_m_s2_{axis_name}"],
            color=COLORS[1],
            linestyle="--",
            label="ESO estimate",
        )
        top.set_title(f"Position ESO {axis_name.upper()}")
        top.set_ylabel("Acceleration [m/s²]")

        bottom = axes[1, index]
        bottom.plot(
            attitude["time_s"],
            attitude[f"true_disturbance_frd_rad_s2_{axis_name}"],
            color=COLORS[0],
            label="true synthetic disturbance",
        )
        bottom.plot(
            attitude["time_s"],
            attitude[f"disturbance_hat_frd_rad_s2_{axis_name}"],
            color=COLORS[1],
            linestyle="--",
            label="ESO estimate",
        )
        bottom.set_title(f"Attitude ESO {axis_name.upper()}")
        bottom.set_ylabel("Angular accel. [rad/s²]")
    _finish_axes(axes)
    _add_watermark(figure, fixture_watermark)
    return _save_figure(figure, output_dir, "m2_eso_estimates")


def _plot_sliding(
    position: Mapping[str, np.ndarray],
    attitude: Mapping[str, np.ndarray],
    fixture_watermark: str,
    output_dir: Path,
) -> List[Path]:
    figure, axes = _new_figure("M2 sliding vectors")
    for index, axis_name in enumerate(AXES):
        top = axes[0, index]
        top.plot(
            position["time_s"],
            position[f"sliding_ned_m_s_{axis_name}"],
            color=COLORS[2],
            label="position sliding vector",
        )
        top.axhline(0.0, color="#333333", linewidth=0.7)
        top.set_title(f"Position {axis_name.upper()}")
        top.set_ylabel("Sliding [m/s]")

        bottom = axes[1, index]
        bottom.plot(
            attitude["time_s"],
            attitude[f"sliding_s_inv_{axis_name}"],
            color=COLORS[3],
            label="attitude sliding vector",
        )
        bottom.axhline(0.0, color="#333333", linewidth=0.7)
        bottom.set_title(f"Attitude {axis_name.upper()}")
        bottom.set_ylabel("Sliding [s⁻¹]")
    _finish_axes(axes)
    _add_watermark(figure, fixture_watermark)
    return _save_figure(figure, output_dir, "m2_sliding_vectors")


def _plot_control(
    position: Mapping[str, np.ndarray],
    attitude: Mapping[str, np.ndarray],
    fixture_watermark: str,
    output_dir: Path,
) -> List[Path]:
    figure, axes = _new_figure("M2 physical control inputs (unsaturated reference)")
    for index, axis_name in enumerate(AXES):
        top = axes[0, index]
        top.plot(
            position["time_s"],
            position[f"force_ned_n_{axis_name}"],
            color=COLORS[0],
            label="force command",
        )
        top.set_title(f"NED force {axis_name.upper()}")
        top.set_ylabel("Force [N]")

        bottom = axes[1, index]
        bottom.plot(
            attitude["time_s"],
            attitude[f"torque_frd_n_m_{axis_name}"],
            color=COLORS[1],
            label="torque command",
        )
        bottom.set_title(f"FRD torque {axis_name.upper()}")
        bottom.set_ylabel("Torque [N·m]")
    _finish_axes(axes)
    _add_watermark(figure, fixture_watermark)
    return _save_figure(figure, output_dir, "m2_control_inputs")


def generate_reproduction_plots(data_dir: Path, output_dir: Path) -> Path:
    """从版本化闭环向量生成四组 PNG/SVG 图及确定性哈希清单。"""

    manifest_path = data_dir / "closed_loop_v1_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    position_path = data_dir / "closed_loop_position_v1.csv"
    attitude_path = data_dir / "closed_loop_attitude_v1.csv"
    if _sha256(position_path) != manifest["position"]["csv_sha256"]:
        raise ValueError("位置闭环 CSV 哈希与清单不一致")
    if _sha256(attitude_path) != manifest["attitude"]["csv_sha256"]:
        raise ValueError("姿态闭环 CSV 哈希与清单不一致")
    position_columns, position = _read_csv(position_path)
    attitude_columns, attitude = _read_csv(attitude_path)
    if position_columns != manifest["position"]["columns"]:
        raise ValueError("位置闭环 CSV 列顺序与清单不一致")
    if attitude_columns != manifest["attitude"]["columns"]:
        raise ValueError("姿态闭环 CSV 列顺序与清单不一致")

    output_dir.mkdir(parents=True, exist_ok=True)
    _configure_style()
    fixture_watermark = manifest["fixture"]["watermark"]
    assets: List[Path] = []
    assets.extend(_plot_tracking(position, attitude, fixture_watermark, output_dir))
    assets.extend(_plot_eso(position, attitude, fixture_watermark, output_dir))
    assets.extend(_plot_sliding(position, attitude, fixture_watermark, output_dir))
    assets.extend(_plot_control(position, attitude, fixture_watermark, output_dir))

    plot_manifest = {
        "schema_version": 1,
        "dataset_id": manifest["dataset_id"],
        "watermarks": [WATERMARK, fixture_watermark],
        "backend": matplotlib.get_backend(),
        "matplotlib": matplotlib.__version__,
        "figure_size_inches": list(FIGSIZE_INCH),
        "dpi": DPI,
        "pixel_size": [int(FIGSIZE_INCH[0] * DPI), int(FIGSIZE_INCH[1] * DPI)],
        "source_sha256": {
            "plot_reproduction": _sha256(Path(__file__)),
            "closed_loop_manifest": _sha256(manifest_path),
            "closed_loop_position": _sha256(position_path),
            "closed_loop_attitude": _sha256(attitude_path),
        },
        "assets_sha256": {path.name: _sha256(path) for path in assets},
    }
    output_manifest = output_dir / "m2_plot_manifest.json"
    output_manifest.write_text(
        json.dumps(plot_manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return output_manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-dir", type=Path, default=REPO_ROOT / "test/pregme/data"
    )
    parser.add_argument(
        "--output-dir", type=Path, default=REPO_ROOT / "docs/figures/m2"
    )
    args = parser.parse_args()
    manifest = generate_reproduction_plots(args.data_dir, args.output_dir)
    print(f"plot manifest: {manifest}")
    print(f"figures: {len(FIGURE_STEMS)} PNG + {len(FIGURE_STEMS)} SVG")
    print(f"watermark: {WATERMARK}")


if __name__ == "__main__":
    main()
