"""论文式 (13)、式 (14) 姿态变增益 ESO 测试。"""

import numpy as np
import pytest

from tools.pregme_reference.attitude import AttitudeEso
from tools.pregme_reference.attitude import AttitudeEsoConfig
from tools.pregme_reference.attitude import attitude_eso_config_from_paper
from tools.pregme_reference.attitude import rigid_body_known_angular_acceleration
from tools.pregme_reference.equations import load_paper_default
from tools.pregme_reference.eso import EsoUpdateStatus
from tools.pregme_reference.eso import PositionEso
from tools.pregme_reference.eso import PositionEsoConfig
from tools.pregme_reference.generate_attitude_golden_vectors import (
    generate_attitude_scenario,
)


def _paper_config(minimum_period_s=0.005, maximum_period_s=0.02):
    return attitude_eso_config_from_paper(
        load_paper_default(), minimum_period_s, maximum_period_s
    )


def _shared_config(config):
    return PositionEsoConfig(
        alpha=config.alpha,
        epsilon=config.epsilon,
        w=config.w,
        d=config.d,
        minimum_period_s=config.minimum_period_s,
        maximum_period_s=config.maximum_period_s,
    )


def test_m2_att_01_initial_sample_matches_angular_velocity_exactly():
    observer = AttitudeEso(_paper_config())
    omega = np.array([0.12, -0.08, 0.05])
    update = observer.update(omega, np.zeros(3), 1_000_000)
    assert update.status is EsoUpdateStatus.INITIALIZED
    assert update.valid is True
    assert update.integrated is False
    assert update.estimate is not None
    np.testing.assert_array_equal(update.estimate.h_omega_frd_rad_s, omega)
    np.testing.assert_array_equal(update.estimate.innovation_frd_rad_s, np.zeros(3))
    np.testing.assert_array_equal(
        update.estimate.disturbance_hat_frd_rad_s2, np.zeros(3)
    )


def test_m2_att_02_wrapper_is_numerically_identical_to_shared_eso_core():
    config = _paper_config()
    attitude = AttitudeEso(config)
    shared = PositionEso(_shared_config(config))
    samples = [
        (1_000_000, [0.1, -0.2, 0.3], [0.2, -0.1, 0.05]),
        (1_010_000, [0.11, -0.19, 0.32], [0.21, -0.09, 0.04]),
        (1_020_000, [0.13, -0.18, 0.31], [0.19, -0.08, 0.03]),
    ]
    for timestamp, omega, known_input in samples:
        actual = attitude.update(omega, known_input, timestamp)
        expected = shared.update(omega, known_input, timestamp)
        assert actual.status is expected.status
        assert actual.valid is expected.valid
        assert actual.integrated is expected.integrated
        assert actual.estimate is not None
        assert expected.estimate is not None
        np.testing.assert_array_equal(
            actual.estimate.h_omega_frd_rad_s, expected.estimate.h_v_ned_m_s
        )
        np.testing.assert_array_equal(
            actual.estimate.innovation_frd_rad_s,
            expected.estimate.innovation_ned_m_s,
        )
        np.testing.assert_array_equal(
            actual.estimate.disturbance_hat_frd_rad_s2,
            expected.estimate.disturbance_hat_ned_m_s2,
        )


def test_m2_att_03_zero_rate_reduces_to_inertia_solve():
    inertia = np.diag([0.48, 0.48, 0.70])
    torque = np.array([0.48, -0.96, 2.10])
    actual = rigid_body_known_angular_acceleration(np.zeros(3), torque, inertia)
    np.testing.assert_allclose(actual, [1.0, -2.0, 3.0], rtol=0.0, atol=1.0e-15)


def test_m2_att_04_gyroscopic_cross_product_has_rigid_body_sign():
    inertia = np.diag([0.48, 0.48, 0.70])
    omega = np.array([2.0, 3.0, 4.0])
    actual = rigid_body_known_angular_acceleration(omega, np.zeros(3), inertia)
    expected_cross = np.array(
        [
            (0.70 - 0.48) * omega[1] * omega[2],
            (0.48 - 0.70) * omega[0] * omega[2],
            (0.48 - 0.48) * omega[0] * omega[1],
        ]
    )
    np.testing.assert_allclose(inertia @ actual, -expected_cross, rtol=0.0, atol=1e-15)


def test_m2_att_05_inertia_scaling_halves_zero_rate_acceleration():
    inertia = np.diag([0.48, 0.48, 0.70])
    torque = np.array([0.1, -0.2, 0.3])
    original = rigid_body_known_angular_acceleration(np.zeros(3), torque, inertia)
    doubled = rigid_body_known_angular_acceleration(np.zeros(3), torque, 2.0 * inertia)
    np.testing.assert_allclose(doubled, 0.5 * original, rtol=0.0, atol=1e-15)


@pytest.mark.parametrize(
    "inertia,match",
    [
        (np.eye(2), "形状"),
        (np.array([[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]), "对称"),
        (np.diag([1.0, 1.0, 0.0]), "正定"),
        (np.diag([1.0, np.nan, 1.0]), "有限"),
    ],
)
def test_m2_att_06_rejects_invalid_inertia(inertia, match):
    with pytest.raises(ValueError, match=match):
        rigid_body_known_angular_acceleration(np.zeros(3), np.zeros(3), inertia)


def test_m2_att_07_timing_fault_and_reset_semantics_match_shared_state_machine():
    config = _paper_config()
    observer = AttitudeEso(config)
    first = observer.update(np.ones(3), np.zeros(3), 1_000_000)
    duplicate = observer.update(np.full(3, np.nan), np.full(3, np.nan), 1_000_000)
    assert first.valid is True
    assert duplicate.status is EsoUpdateStatus.DUPLICATE
    assert duplicate.valid is True

    cases = [
        (0, np.zeros(3), np.zeros(3), EsoUpdateStatus.INVALID_ZERO_TIMESTAMP),
        (999_999, np.zeros(3), np.zeros(3), EsoUpdateStatus.TIMESTAMP_REGRESSION),
        (1_001_000, np.zeros(3), np.zeros(3), EsoUpdateStatus.DT_OUT_OF_RANGE),
        (1_030_000, np.zeros(3), np.zeros(3), EsoUpdateStatus.DT_OUT_OF_RANGE),
        (1_010_000, [0.0, 0.0], np.zeros(3), EsoUpdateStatus.INVALID_SHAPE),
        (
            1_010_000,
            np.zeros(3),
            [0.0, np.inf, 0.0],
            EsoUpdateStatus.NONFINITE_INPUT,
        ),
    ]
    for timestamp, omega, known_input, expected_status in cases:
        observer = AttitudeEso(config)
        observer.update(np.zeros(3), np.zeros(3), 1_000_000)
        fault = observer.update(omega, known_input, timestamp)
        assert fault.status is expected_status
        assert fault.estimate is None
        assert observer.needs_reset is True
        blocked = observer.update(np.ones(3), np.zeros(3), 1_010_000)
        assert blocked.status is EsoUpdateStatus.NEEDS_RESET
        observer.reset()
        recovered = observer.update([0.2, 0.1, -0.1], np.zeros(3), 2_000_000)
        assert recovered.status is EsoUpdateStatus.INITIALIZED


def test_m2_att_08_constant_z_disturbance_converges_with_alignment_bias():
    scenario = generate_attitude_scenario("constant")
    final_error = np.abs(
        scenario.disturbance_hat_frd_rad_s2[-1]
        - scenario.true_disturbance_frd_rad_s2[-1]
    )
    assert np.max(final_error) < 1.1e-2
    assert scenario.disturbance_hat_frd_rad_s2[-1, 2] == pytest.approx(
        1.01013229, rel=0.0, abs=1.0e-8
    )


def test_m2_att_09_step_z_disturbance_is_monotonic_and_has_no_overshoot():
    scenario = generate_attitude_scenario("step")
    post_step = scenario.disturbance_hat_frd_rad_s2[500:, 2]
    assert np.all(np.diff(post_step) >= 0.0)
    assert np.all(post_step <= post_step[-1] * (1.0 + 1.0e-12))
    assert post_step[-1] == pytest.approx(1.51933958, rel=0.0, abs=1.0e-8)
    assert np.all(np.isfinite(post_step))
    np.testing.assert_array_equal(
        scenario.disturbance_hat_frd_rad_s2[:, :2],
        np.zeros_like(scenario.disturbance_hat_frd_rad_s2[:, :2]),
    )


def test_m2_att_10_distinct_axes_match_three_independent_shared_channels():
    config = AttitudeEsoConfig(
        alpha=[0.5, 1.0, 1.5],
        epsilon=[0.2, 0.4, 0.8],
        w=0.5,
        d=5.0,
        minimum_period_s=0.005,
        maximum_period_s=0.02,
    )
    vector = AttitudeEso(config)
    vector.update([0.1, -0.2, 0.3], [0.2, -0.1, 0.05], 1_000_000)
    result = vector.update([0.12, -0.18, 0.34], [0.1, -0.2, 0.04], 1_010_000)
    assert result.estimate is not None
    independent = np.empty(3)
    for axis in range(3):
        scalar_config = AttitudeEsoConfig(
            alpha=np.full(3, config.alpha[axis]),
            epsilon=np.full(3, config.epsilon[axis]),
            w=config.w,
            d=config.d,
            minimum_period_s=config.minimum_period_s,
            maximum_period_s=config.maximum_period_s,
        )
        scalar = AttitudeEso(scalar_config)
        scalar.update(
            np.full(3, [0.1, -0.2, 0.3][axis]),
            np.full(3, [0.2, -0.1, 0.05][axis]),
            1_000_000,
        )
        scalar_result = scalar.update(
            np.full(3, [0.12, -0.18, 0.34][axis]),
            np.full(3, [0.1, -0.2, 0.04][axis]),
            1_010_000,
        )
        assert scalar_result.estimate is not None
        independent[axis] = scalar_result.estimate.disturbance_hat_frd_rad_s2[0]
    np.testing.assert_allclose(
        result.estimate.disturbance_hat_frd_rad_s2,
        independent,
        rtol=1.0e-15,
        atol=0.0,
    )


def test_m2_att_11_sine_tracking_and_seeded_noise_are_deterministic():
    sine = generate_attitude_scenario("sine")
    steady = sine.time_s >= 60.0
    correlations = [
        np.corrcoef(
            sine.disturbance_hat_frd_rad_s2[steady, axis],
            sine.true_disturbance_frd_rad_s2[steady, axis],
        )[0, 1]
        for axis in range(3)
    ]
    assert np.all(np.asarray(correlations) > 0.7)

    noise_a = generate_attitude_scenario("noise", seed=20261118)
    noise_b = generate_attitude_scenario("noise", seed=20261118)
    noise_c = generate_attitude_scenario("noise", seed=20261119)
    np.testing.assert_array_equal(
        noise_a.angular_velocity_frd_rad_s, noise_b.angular_velocity_frd_rad_s
    )
    assert not np.array_equal(
        noise_a.angular_velocity_frd_rad_s, noise_c.angular_velocity_frd_rad_s
    )
    np.testing.assert_array_equal(noise_a.torque_frd_n_m, noise_c.torque_frd_n_m)
    np.testing.assert_array_equal(
        noise_a.true_disturbance_frd_rad_s2,
        noise_c.true_disturbance_frd_rad_s2,
    )


def test_m2_att_12_inputs_unchanged_outputs_read_only_and_epsilon_is_omega():
    parameters = load_paper_default()
    config = _paper_config()
    np.testing.assert_array_equal(config.epsilon, parameters.attitude.eso.epsilon)
    assert not np.array_equal(config.epsilon, parameters.position.eso.epsilon)
    observer = AttitudeEso(config)
    omega = np.array([0.1, -0.2, 0.3])
    known_input = np.array([0.2, -0.1, 0.05])
    omega_before = omega.copy()
    input_before = known_input.copy()
    update = observer.update(omega, known_input, 1_000_000)
    np.testing.assert_array_equal(omega, omega_before)
    np.testing.assert_array_equal(known_input, input_before)
    assert update.estimate is not None
    assert update.estimate.h_omega_frd_rad_s.flags.writeable is False
    assert update.estimate.innovation_frd_rad_s.flags.writeable is False
    assert update.estimate.disturbance_hat_frd_rad_s2.flags.writeable is False
