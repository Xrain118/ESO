"""性能包络和预设误差轨迹的边界与导数测试。"""

import warnings

import numpy as np
import pytest

from tools.pregme_reference.envelope import envelope
from tools.pregme_reference.envelope import envelope_dot
from tools.pregme_reference.envelope import preset_trajectory
from tools.pregme_reference.envelope import preset_trajectory_b
from tools.pregme_reference.envelope import preset_trajectory_ddot
from tools.pregme_reference.envelope import preset_trajectory_dot


RHO0 = np.array([3.0, 2.0, 1.0])
RHO_INF = np.array([0.05, 0.04, 0.03])
L_AXIS = np.array([1.0, 1.5, 0.8])
BETA0 = np.array([0.8, -0.5, 0.25])
BETA_DOT0 = np.array([-0.2, 0.1, 0.05])
C_AXIS = np.array([5.0, 4.0, 6.0])


def _five_point_derivative(function, t, h):
    return (
        function(t - 2.0 * h)
        - 8.0 * function(t - h)
        + 8.0 * function(t + h)
        - function(t + 2.0 * h)
    ) / (12.0 * h)


def test_m2_env_01_initial_value():
    np.testing.assert_allclose(
        envelope(0.0, RHO0, RHO_INF, L_AXIS), RHO0, rtol=0.0, atol=1.0e-15
    )


def test_m2_env_02a_ten_time_constants_matches_exact_residual():
    rho0 = np.array([3.0, 3.0, 3.0])
    rho_inf = np.array([0.05, 0.05, 0.05])
    actual = envelope(10.0, rho0, rho_inf, 1.0)
    normalized = (actual - rho_inf) / (rho0 - rho_inf)
    np.testing.assert_allclose(
        normalized, np.full(3, np.exp(-10.0)), rtol=1.0e-12, atol=1.0e-15
    )


def test_m2_env_02b_late_value_reaches_absolute_tolerance():
    rho0 = np.array([3.0, 3.0, 3.0])
    rho_inf = np.array([0.05, 0.05, 0.05])
    actual = envelope(30.0, rho0, rho_inf, 1.0)
    assert np.all(np.abs(actual - rho_inf) < 1.0e-9)


def test_m2_env_03_is_strictly_decreasing():
    time = np.linspace(0.0, 12.0, 241)
    values = envelope(time, RHO0, RHO_INF, L_AXIS)
    derivatives = envelope_dot(time, RHO0, RHO_INF, L_AXIS)
    assert np.all(derivatives < 0.0)
    assert np.all(np.diff(values, axis=0) < 0.0)


@pytest.mark.parametrize("time", [0.1, 0.5, 1.0, 2.0, 5.0])
def test_m2_env_04_analytic_derivative_matches_five_point_difference(time):
    h = 1.0e-6 * max(1.0, time)
    numerical = _five_point_derivative(
        lambda value: envelope(value, RHO0, RHO_INF, L_AXIS), time, h
    )
    analytic = envelope_dot(time, RHO0, RHO_INF, L_AXIS)
    np.testing.assert_allclose(numerical, analytic, rtol=1.0e-9, atol=1.0e-12)


@pytest.mark.parametrize(
    "rho0,rho_inf,l,field",
    [
        ([0.05, 2.0, 1.0], RHO_INF, L_AXIS, "rho0"),
        (RHO0, [0.0, 0.04, 0.03], L_AXIS, "rho_inf"),
        (RHO0, RHO_INF, [0.0, 1.5, 0.8], "l"),
        (RHO0, RHO_INF, [np.nan, 1.5, 0.8], "l"),
    ],
)
def test_m2_env_05_rejects_invalid_parameters(rho0, rho_inf, l, field):
    with pytest.raises(ValueError, match=field):
        envelope(0.0, rho0, rho_inf, l)


def test_m2_bet_01_initial_value_and_derivative():
    np.testing.assert_allclose(
        preset_trajectory(0.0, BETA0, BETA_DOT0, L_AXIS, C_AXIS),
        BETA0,
        rtol=0.0,
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        preset_trajectory_dot(0.0, BETA0, BETA_DOT0, L_AXIS, C_AXIS),
        BETA_DOT0,
        rtol=0.0,
        atol=1.0e-9,
    )


def test_m2_bet_02a_ten_time_constants_matches_rearranged_formula():
    beta0 = np.array([1.0, -0.5, 0.25])
    beta_dot0 = np.zeros(3)
    l_value = 1.0
    c_value = np.array([5.0, 5.0, 5.0])
    b_value = l_value * beta0 + beta_dot0
    time = 10.0
    expected = (beta0 + b_value / c_value) * np.exp(-l_value * time) - (
        b_value / c_value
    ) * np.exp(-(l_value + c_value) * time)
    actual = preset_trajectory(time, beta0, beta_dot0, l_value, c_value)
    np.testing.assert_allclose(actual, expected, rtol=1.0e-12, atol=1.0e-15)


def test_m2_bet_02b_late_value_reaches_relative_scale():
    beta0 = np.array([1.0, -0.5, 0.25])
    actual = preset_trajectory(20.0, beta0, np.zeros(3), 1.0, [5.0, 5.0, 5.0])
    assert np.all(np.abs(actual) < 1.0e-6 * np.max(np.abs(beta0)))


@pytest.mark.parametrize("time", [0.1, 0.5, 1.0, 2.0, 5.0])
def test_m2_bet_03_analytic_derivatives_match_five_point_difference(time):
    h = 1.0e-6 * max(1.0, time)
    numerical_dot = _five_point_derivative(
        lambda value: preset_trajectory(value, BETA0, BETA_DOT0, L_AXIS, C_AXIS),
        time,
        h,
    )
    numerical_ddot = _five_point_derivative(
        lambda value: preset_trajectory_dot(
            value, BETA0, BETA_DOT0, L_AXIS, C_AXIS
        ),
        time,
        h,
    )
    analytic_dot = preset_trajectory_dot(time, BETA0, BETA_DOT0, L_AXIS, C_AXIS)
    analytic_ddot = preset_trajectory_ddot(time, BETA0, BETA_DOT0, L_AXIS, C_AXIS)
    np.testing.assert_allclose(
        numerical_dot, analytic_dot, rtol=1.0e-9, atol=1.0e-11
    )
    np.testing.assert_allclose(
        numerical_ddot, analytic_ddot, rtol=1.0e-9, atol=1.0e-10
    )


def test_m2_bet_04_small_c_uses_removable_singularity_limit_without_warning():
    beta0 = np.array([0.7, -0.4, 0.2])
    beta_dot0 = np.array([0.1, -0.2, 0.05])
    l_axis = np.array([0.8, 1.1, 1.4])
    c_axis = np.full(3, 1.0e-8)
    time = np.array([0.0, 0.2, 1.0, 3.0])
    b_axis = l_axis * beta0 + beta_dot0
    expected = (beta0[None, :] + time[:, None] * b_axis[None, :]) * np.exp(
        -time[:, None] * l_axis[None, :]
    )
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        actual = preset_trajectory(time, beta0, beta_dot0, l_axis, c_axis)
        actual_dot = preset_trajectory_dot(time, beta0, beta_dot0, l_axis, c_axis)
        actual_ddot = preset_trajectory_ddot(time, beta0, beta_dot0, l_axis, c_axis)
    np.testing.assert_allclose(actual, expected, rtol=1.0e-9, atol=1.0e-15)
    assert np.all(np.isfinite(actual_dot))
    assert np.all(np.isfinite(actual_ddot))


def test_m2_bet_05_three_axes_are_independent():
    time = np.array([0.0, 0.3, 1.2])
    c_axis = np.array([2.0, 5.0, 9.0])
    b_axis = preset_trajectory_b(BETA0, BETA_DOT0, L_AXIS)
    actual = preset_trajectory(time, BETA0, BETA_DOT0, L_AXIS, c_axis)
    expected = np.empty_like(actual)
    for axis in range(3):
        expected[:, axis] = (
            BETA0[axis] * np.exp(-L_AXIS[axis] * time)
            + b_axis[axis]
            / c_axis[axis]
            * (1.0 - np.exp(-c_axis[axis] * time))
            * np.exp(-L_AXIS[axis] * time)
        )
    np.testing.assert_allclose(actual, expected, rtol=1.0e-15, atol=1.0e-15)


@pytest.mark.parametrize(
    "time,l,c,field",
    [
        (0.0, L_AXIS, [0.0, 4.0, 6.0], "c"),
        (0.0, [0.0, 1.5, 0.8], C_AXIS, "l"),
        (-0.1, L_AXIS, C_AXIS, "t"),
        (np.nan, L_AXIS, C_AXIS, "t"),
        (0.0, L_AXIS, [np.inf, 4.0, 6.0], "c"),
    ],
)
def test_m2_bet_06_rejects_invalid_input(time, l, c, field):
    with pytest.raises(ValueError, match=field):
        preset_trajectory(time, BETA0, BETA_DOT0, l, c)


def test_m2_shape_01_scalar_and_vector_time_shapes_match():
    scalar_time = 0.5
    vector_time = np.array([scalar_time])
    functions = [
        lambda time: envelope(time, RHO0, RHO_INF, L_AXIS),
        lambda time: envelope_dot(time, RHO0, RHO_INF, L_AXIS),
        lambda time: preset_trajectory(time, BETA0, BETA_DOT0, L_AXIS, C_AXIS),
        lambda time: preset_trajectory_dot(time, BETA0, BETA_DOT0, L_AXIS, C_AXIS),
        lambda time: preset_trajectory_ddot(time, BETA0, BETA_DOT0, L_AXIS, C_AXIS),
    ]
    for function in functions:
        scalar_result = function(scalar_time)
        vector_result = function(vector_time)
        assert scalar_result.shape == (3,)
        assert vector_result.shape == (1, 3)
        assert scalar_result.dtype == np.float64
        assert vector_result.dtype == np.float64
        np.testing.assert_array_equal(vector_result[0], scalar_result)
