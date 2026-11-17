"""论文参数加载与式 (4) 变增益测试。"""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from tools.pregme_reference.envelope import envelope
from tools.pregme_reference.equations import load_paper_default
from tools.pregme_reference.equations import paper_default_sha256
from tools.pregme_reference.equations import validate_parameters
from tools.pregme_reference.equations import variable_gain


REPO_ROOT = Path(__file__).resolve().parents[2]
PAPER_PATH = REPO_ROOT / "config/paper_default.yaml"


def test_m2_gain_01_zero_point():
    assert variable_gain(0.0, 0.5, 5.0) == np.float64(0.0)


@pytest.mark.parametrize("error", [1.0e-9, 0.1, 3.0, 40.0])
def test_m2_gain_02_odd_function(error):
    positive = variable_gain(error, 0.5, 5.0)
    negative = variable_gain(-error, 0.5, 5.0)
    assert negative == pytest.approx(-positive, rel=1.0e-15, abs=0.0)


@pytest.mark.parametrize("error", [1.0e3, 1.0e6])
def test_m2_gain_03_large_error_is_finite(error):
    result = variable_gain(error, 0.5, 5.0)
    assert np.isfinite(result)
    assert result == pytest.approx(error / 0.5, rel=1.0e-12, abs=0.0)


def test_m2_gain_04_small_error_slope_matches_equation_4():
    error = 1.0e-12
    result = variable_gain(error, 0.5, 5.0)
    expected = error / (0.5 + 5.0 / 2.0)
    assert result == pytest.approx(expected, rel=1.0e-12, abs=0.0)


def test_m2_gain_05_matches_naive_formula_in_safe_range():
    errors = np.array([-20.0, -3.0, -0.1, 0.0, 0.1, 3.0, 20.0])
    exponent_sum = np.exp(errors) + np.exp(-errors)
    expected = errors * exponent_sum / (0.5 * exponent_sum + 5.0)
    actual = variable_gain(errors, 0.5, 5.0)
    np.testing.assert_allclose(actual, expected, rtol=1.0e-12, atol=1.0e-15)

    large = variable_gain(np.array([-1.0e6, -351.0, 351.0, 1.0e6]), 0.5, 5.0)
    assert np.all(np.isfinite(large))


@pytest.mark.parametrize(
    "error,w,d,field",
    [
        (0.0, 0.0, 5.0, "w"),
        (0.0, -1.0, 5.0, "w"),
        (0.0, 0.5, -1.0, "d"),
        (np.nan, 0.5, 5.0, "e"),
        (np.inf, 0.5, 5.0, "e"),
    ],
)
def test_m2_gain_06_rejects_invalid_input(error, w, d, field):
    with pytest.raises(ValueError, match=field):
        variable_gain(error, w, d)


def test_m2_load_01_reads_paper_values_and_hash():
    parameters = load_paper_default(PAPER_PATH)
    assert parameters.nominal_platform.quadcopter_mass_kg == pytest.approx(
        5.40, rel=0.0, abs=0.0
    )
    assert parameters.nominal_platform.robotic_arm_mass_kg == pytest.approx(
        2.32, rel=0.0, abs=0.0
    )
    assert parameters.nominal_platform.wheelbase_m == pytest.approx(
        0.90, rel=0.0, abs=0.0
    )
    np.testing.assert_array_equal(
        parameters.position.performance_envelope.rho_0,
        np.array([3.0, 3.0, 3.0]),
    )
    np.testing.assert_array_equal(
        parameters.position.control.k_diagonal,
        np.array([3.0, 3.0, 3.0]),
    )
    digest = paper_default_sha256(PAPER_PATH)
    assert len(digest) == 64
    assert digest == "6f7559bad4f0f3e28a81b2cbfa0d552bc51c8b024d6b509a1bb0bd7780b2bf8b"


def test_m2_load_02_yaml_copy_changes_computed_result(tmp_path):
    with PAPER_PATH.open(encoding="utf-8") as stream:
        modified = yaml.safe_load(stream)
    modified["position"]["performance_envelope"]["rho_0_m"][0] = 4.0
    copied_path = tmp_path / "paper_default_modified.yaml"
    with copied_path.open("w", encoding="utf-8", newline="\n") as stream:
        yaml.safe_dump(modified, stream, allow_unicode=True, sort_keys=False)

    original = load_paper_default(PAPER_PATH)
    copied = load_paper_default(copied_path)
    original_value = envelope(
        0.5,
        original.position.performance_envelope.rho_0,
        original.position.performance_envelope.rho_inf,
        original.position.performance_envelope.l,
    )
    copied_value = envelope(
        0.5,
        copied.position.performance_envelope.rho_0,
        copied.position.performance_envelope.rho_inf,
        copied.position.performance_envelope.l,
    )
    assert copied.position.performance_envelope.rho_0[0] == 4.0
    assert copied_value[0] != original_value[0]
    np.testing.assert_array_equal(copied_value[1:], original_value[1:])


def test_parameter_validation_reports_dotted_field_path():
    parameters = load_paper_default(PAPER_PATH)
    invalid_envelope = replace(
        parameters.position.performance_envelope,
        rho_inf=np.array([0.0, 0.05, 0.05], dtype=np.float64),
    )
    invalid_position = replace(parameters.position, performance_envelope=invalid_envelope)
    invalid_parameters = replace(parameters, position=invalid_position)
    with pytest.raises(ValueError, match="position.performance_envelope.rho_inf"):
        validate_parameters(invalid_parameters)
