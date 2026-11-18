"""论文式 (3)、式 (7) 位置变增益 ESO 测试。"""

import numpy as np
import pytest

from tools.pregme_reference.equations import load_paper_default
from tools.pregme_reference.equations import variable_gain
from tools.pregme_reference.eso import EsoUpdateStatus
from tools.pregme_reference.eso import PositionEso
from tools.pregme_reference.eso import PositionEsoConfig
from tools.pregme_reference.eso import position_eso_config_from_paper
from tools.pregme_reference.eso import variable_gain_eso_estimate
from tools.pregme_reference.eso import variable_gain_eso_rk4_step
from tools.pregme_reference.generate_golden_vectors import generate_position_scenario
from tools.pregme_reference.generate_golden_vectors import load_fixture_context


def _paper_config(minimum_period_s=0.005, maximum_period_s=0.02):
    return position_eso_config_from_paper(
        load_paper_default(), minimum_period_s, maximum_period_s
    )


def _run_known_input(dt, duration_s=5.0):
    config = _paper_config(0.001, 0.02)
    observer = PositionEso(config)
    velocity = np.zeros(3)
    known_input = np.array([0.2, -0.1, 0.05])
    timestamp_us = 1_000_000
    update = observer.update(velocity, known_input, timestamp_us)
    for _ in range(int(round(duration_s / dt))):
        velocity = velocity + dt * known_input
        timestamp_us += int(round(dt * 1_000_000.0))
        update = observer.update(velocity, known_input, timestamp_us)
    assert update.estimate is not None
    return update.estimate


def test_m2_eso_01_initial_sample_matches_velocity_exactly():
    observer = PositionEso(_paper_config())
    velocity = np.array([1.0, 2.0, 3.0])
    update = observer.update(velocity, np.zeros(3), 1_000_000)
    assert update.status is EsoUpdateStatus.INITIALIZED
    assert update.valid is True
    assert update.integrated is False
    assert observer.initialized is True
    assert update.estimate is not None
    np.testing.assert_array_equal(update.estimate.h_v_ned_m_s, velocity)
    np.testing.assert_array_equal(update.estimate.innovation_ned_m_s, np.zeros(3))
    np.testing.assert_array_equal(
        update.estimate.disturbance_hat_ned_m_s2, np.zeros(3)
    )


def test_m2_eso_02_disturbance_estimate_uses_current_innovation():
    config = _paper_config()
    observer = PositionEso(config)
    observer.update(np.zeros(3), np.zeros(3), 1_000_000)
    velocity = np.array([0.01, -0.02, 0.03])
    update = observer.update(velocity, np.zeros(3), 1_010_000)
    assert update.estimate is not None
    expected = (
        config.alpha
        * variable_gain(
            velocity - update.estimate.h_v_ned_m_s, config.w, config.d
        )
        / config.epsilon
    )
    np.testing.assert_array_equal(
        update.estimate.disturbance_hat_ned_m_s2, expected
    )


def test_m2_eso_03_zero_case_and_known_input_alignment_error():
    zero = generate_position_scenario("zero")
    np.testing.assert_array_equal(zero.h_v_ned_m_s, zero.velocity_ned_m_s)
    np.testing.assert_array_equal(zero.innovation_ned_m_s, np.zeros_like(zero.innovation_ned_m_s))
    np.testing.assert_array_equal(
        zero.disturbance_hat_ned_m_s2,
        np.zeros_like(zero.disturbance_hat_ned_m_s2),
    )

    estimate_dt = _run_known_input(0.01)
    estimate_half_dt = _run_known_input(0.005)
    error_dt = np.linalg.norm(estimate_dt.disturbance_hat_ned_m_s2)
    error_half_dt = np.linalg.norm(estimate_half_dt.disturbance_hat_ned_m_s2)
    assert error_dt < 3.0e-5
    assert error_half_dt < error_dt
    assert error_half_dt / error_dt == pytest.approx(0.5, rel=0.05, abs=0.0)


def test_m2_eso_04_constant_disturbance_converges_with_discrete_alignment_bias():
    scenario = generate_position_scenario("constant")
    final_error = np.abs(
        scenario.disturbance_hat_ned_m_s2[-1]
        - scenario.true_disturbance_ned_m_s2[-1]
    )
    assert np.max(final_error) < 3.0e-3
    final_window = scenario.disturbance_hat_ned_m_s2[-101:]
    assert np.max(np.ptp(final_window, axis=0)) < 1.0e-8
    assert np.all(np.abs(scenario.innovation_ned_m_s[-1]) > 1.0)


def test_m2_eso_05_step_disturbance_is_monotonic_and_has_no_overshoot():
    scenario = generate_position_scenario("step")
    post_step = scenario.disturbance_hat_ned_m_s2[500:]
    assert np.all(np.diff(post_step, axis=0) >= 0.0)
    assert np.all(post_step <= 1.5 * (1.0 + 1.0e-12))
    assert np.all(np.isfinite(post_step))


def test_m2_eso_06_rk4_matches_linear_closed_form_and_fourth_order_ratio():
    h0 = np.array([0.1, -0.2, 0.3])
    y = np.array([1.0, 0.5, -0.5])
    u = np.array([0.2, -0.1, 0.05])
    alpha = np.array([0.5, 0.4, 0.3])
    epsilon = np.full(3, 0.5)
    w = 0.5
    gain = alpha / (epsilon * w)
    equilibrium = y + u / gain

    def exact(time_s):
        return equilibrium + (h0 - equilibrium) * np.exp(-gain * time_s)

    single_step = variable_gain_eso_rk4_step(
        h0, y, u, alpha, epsilon, w, 0.0, 0.01
    )
    np.testing.assert_allclose(single_step, exact(0.01), rtol=0.0, atol=1.0e-10)

    def integrate(dt):
        state = h0
        for _ in range(int(round(1.0 / dt))):
            state = variable_gain_eso_rk4_step(
                state, y, u, alpha, epsilon, w, 0.0, dt
            )
        return state

    coarse_error = np.linalg.norm(integrate(0.1) - exact(1.0))
    fine_error = np.linalg.norm(integrate(0.05) - exact(1.0))
    assert fine_error / coarse_error == pytest.approx(1.0 / 16.0, rel=0.35, abs=0.0)


def test_m2_eso_07_timestamp_and_runtime_faults_require_explicit_reset():
    config = _paper_config()

    duplicate_observer = PositionEso(config)
    first = duplicate_observer.update([1.0, 2.0, 3.0], np.zeros(3), 1_000_000)
    duplicate = duplicate_observer.update(
        [np.nan, np.nan, np.nan], [np.nan, np.nan, np.nan], 1_000_000
    )
    assert duplicate.status is EsoUpdateStatus.DUPLICATE
    assert duplicate.valid is True
    assert duplicate.estimate is first.estimate

    cases = [
        (0, np.zeros(3), np.zeros(3), EsoUpdateStatus.INVALID_ZERO_TIMESTAMP),
        (999_999, np.zeros(3), np.zeros(3), EsoUpdateStatus.TIMESTAMP_REGRESSION),
        (1_001_000, np.zeros(3), np.zeros(3), EsoUpdateStatus.DT_OUT_OF_RANGE),
        (1_030_000, np.zeros(3), np.zeros(3), EsoUpdateStatus.DT_OUT_OF_RANGE),
        (1_010_000, [0.0, 0.0], np.zeros(3), EsoUpdateStatus.INVALID_SHAPE),
        (
            1_010_000,
            [0.0, np.inf, 0.0],
            np.zeros(3),
            EsoUpdateStatus.NONFINITE_INPUT,
        ),
    ]
    for timestamp, velocity, known_input, expected_status in cases:
        observer = PositionEso(config)
        observer.update(np.zeros(3), np.zeros(3), 1_000_000)
        fault = observer.update(velocity, known_input, timestamp)
        assert fault.status is expected_status
        assert fault.valid is False
        assert fault.estimate is None
        assert observer.needs_reset is True
        blocked = observer.update([4.0, 5.0, 6.0], np.zeros(3), 2_000_000)
        assert blocked.status is EsoUpdateStatus.NEEDS_RESET
        observer.reset()
        recovered = observer.update([4.0, 5.0, 6.0], np.zeros(3), 2_000_000)
        assert recovered.status is EsoUpdateStatus.INITIALIZED
        assert recovered.estimate is not None
        np.testing.assert_array_equal(recovered.estimate.h_v_ned_m_s, [4.0, 5.0, 6.0])


def test_m2_eso_08_three_channels_match_independent_scalar_replications():
    h = np.array([0.1, -0.2, 0.3])
    y = np.array([1.0, 0.5, -0.5])
    u = np.array([0.2, -0.1, 0.05])
    alpha = np.array([0.1, 0.2, 0.3])
    epsilon = np.array([0.25, 0.5, 0.75])
    vector_result = variable_gain_eso_rk4_step(
        h, y, u, alpha, epsilon, 0.5, 5.0, 0.01
    )
    independent = np.empty(3)
    for axis in range(3):
        replicated = variable_gain_eso_rk4_step(
            np.full(3, h[axis]),
            np.full(3, y[axis]),
            np.full(3, u[axis]),
            np.full(3, alpha[axis]),
            np.full(3, epsilon[axis]),
            0.5,
            5.0,
            0.01,
        )
        independent[axis] = replicated[0]
    np.testing.assert_allclose(vector_result, independent, rtol=1.0e-15, atol=0.0)


@pytest.mark.parametrize("error", [1.0e3, 1.0e6])
def test_m2_eso_09_large_innovation_remains_finite(error):
    alpha = np.array([0.1, 0.2, 0.3])
    epsilon = np.array([0.5, 0.5, 0.5])
    _, disturbance_hat = variable_gain_eso_estimate(
        np.zeros(3), np.full(3, error), alpha, epsilon, 0.5, 5.0
    )
    expected = alpha * (error / 0.5) / epsilon
    assert np.all(np.isfinite(disturbance_hat))
    np.testing.assert_allclose(disturbance_hat, expected, rtol=1.0e-12, atol=0.0)


@pytest.mark.parametrize(
    "field,kwargs",
    [
        ("alpha", {"alpha": [0.0, 0.1, 0.1]}),
        ("epsilon", {"epsilon": [0.0, 0.5, 0.5]}),
        ("epsilon", {"epsilon": [1.0, 0.5, 0.5]}),
        ("w", {"w": 0.0}),
        ("d", {"d": -1.0}),
        ("minimum_period_s", {"minimum_period_s": 0.0}),
        ("maximum_period_s", {"maximum_period_s": 0.004}),
    ],
)
def test_m2_eso_10_rejects_invalid_configuration(field, kwargs):
    values = {
        "alpha": [0.1, 0.1, 0.1],
        "epsilon": [0.5, 0.5, 0.5],
        "w": 0.5,
        "d": 5.0,
        "minimum_period_s": 0.005,
        "maximum_period_s": 0.02,
    }
    values.update(kwargs)
    with pytest.raises(ValueError, match=field):
        PositionEsoConfig(**values)


def test_m2_eso_11_inputs_are_unchanged_and_outputs_are_read_only():
    observer = PositionEso(_paper_config())
    velocity = np.array([1.0, 2.0, 3.0])
    known_input = np.array([0.2, -0.1, 0.05])
    velocity_before = velocity.copy()
    input_before = known_input.copy()
    observer.update(velocity, known_input, 1_000_000)
    update = observer.update(velocity, known_input, 1_010_000)
    np.testing.assert_array_equal(velocity, velocity_before)
    np.testing.assert_array_equal(known_input, input_before)
    assert update.estimate is not None
    assert update.estimate.h_v_ned_m_s.flags.writeable is False
    assert update.estimate.innovation_ned_m_s.flags.writeable is False
    assert update.estimate.disturbance_hat_ned_m_s2.flags.writeable is False


def test_m2_eso_12_sine_tracking_and_seeded_noise_are_deterministic():
    sine = generate_position_scenario("sine")
    steady = sine.time_s >= 60.0
    correlations = [
        np.corrcoef(
            sine.disturbance_hat_ned_m_s2[steady, axis],
            sine.true_disturbance_ned_m_s2[steady, axis],
        )[0, 1]
        for axis in range(3)
    ]
    assert np.all(np.asarray(correlations) > 0.7)

    noise_a = generate_position_scenario("noise", seed=20261118)
    noise_b = generate_position_scenario("noise", seed=20261118)
    noise_c = generate_position_scenario("noise", seed=20261119)
    np.testing.assert_array_equal(noise_a.velocity_ned_m_s, noise_b.velocity_ned_m_s)
    np.testing.assert_array_equal(
        noise_a.disturbance_hat_ned_m_s2, noise_b.disturbance_hat_ned_m_s2
    )
    assert not np.array_equal(noise_a.velocity_ned_m_s, noise_c.velocity_ned_m_s)
    assert np.all(np.isfinite(noise_a.disturbance_hat_ned_m_s2))


def test_fixture_context_matches_frozen_period_window():
    fixture = load_fixture_context()
    assert fixture.expected_position_period_s == pytest.approx(0.01, rel=0.0, abs=0.0)
    assert fixture.minimum_period_s == pytest.approx(0.005, rel=0.0, abs=0.0)
    assert fixture.maximum_period_s == pytest.approx(0.02, rel=0.0, abs=0.0)
