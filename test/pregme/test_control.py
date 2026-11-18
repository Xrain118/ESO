"""论文式 (8)、(9)、(15)–(21) 滑模误差与控制律测试。"""

import numpy as np
import pytest

from tools.pregme_reference.control import AttitudeErrorTerms
from tools.pregme_reference.control import PositionControlConfig
from tools.pregme_reference.control import attitude_control_torque
from tools.pregme_reference.control import attitude_error_terms
from tools.pregme_reference.control import desired_attitude
from tools.pregme_reference.control import position_control_config_from_paper
from tools.pregme_reference.control import position_control_force
from tools.pregme_reference.control import position_error_terms
from tools.pregme_reference.control import position_known_acceleration
from tools.pregme_reference.control import skew_matrix
from tools.pregme_reference.envelope import envelope
from tools.pregme_reference.envelope import preset_trajectory
from tools.pregme_reference.envelope import preset_trajectory_ddot
from tools.pregme_reference.envelope import preset_trajectory_dot
from tools.pregme_reference.equations import GRAVITY_M_S2
from tools.pregme_reference.equations import load_paper_default
from tools.pregme_reference.generate_attitude_golden_vectors import (
    load_attitude_fixture_context,
)


def _rotation(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + np.sin(angle) * cross + (1.0 - np.cos(angle)) * (cross @ cross)


def _exp_so3(rotation_vector):
    angle = np.linalg.norm(rotation_vector)
    if angle < 1.0e-14:
        return np.eye(3)
    return _rotation(rotation_vector / angle, angle)


def test_m2_ctrl_pos_01_error_terms_match_independent_definitions():
    position = np.array([0.3, -0.4, -1.0])
    desired_position = np.array([0.8, -0.8, -0.7])
    velocity = np.array([0.1, -0.05, 0.02])
    desired_velocity = np.array([0.02, 0.01, -0.03])
    beta = np.array([-0.4, 0.3, -0.2])
    beta_dot = np.array([0.05, -0.02, 0.01])
    lambda_ = np.array([3.0, 4.0, 5.0])
    actual = position_error_terms(
        position,
        velocity,
        desired_position,
        desired_velocity,
        beta,
        beta_dot,
        lambda_,
    )
    tracking = position - desired_position
    z = tracking - beta
    z_dot = velocity - desired_velocity - beta_dot
    np.testing.assert_array_equal(actual.tracking_error_ned_m, tracking)
    np.testing.assert_array_equal(actual.z_ned_m, z)
    np.testing.assert_array_equal(actual.z_dot_ned_m_s, z_dot)
    np.testing.assert_array_equal(actual.sliding_ned_m_s, z_dot + lambda_ * z)


def test_m2_ctrl_pos_02_numbered_equation_9_signs_match_independent_expansion():
    paper = load_paper_default()
    config = position_control_config_from_paper(paper)
    errors = position_error_terms(
        [0.3, -0.4, -1.0],
        [0.1, -0.05, 0.02],
        [0.8, -0.8, -0.7],
        [0.02, 0.01, -0.03],
        [-0.4, 0.3, -0.2],
        [0.05, -0.02, 0.01],
        config.lambda_,
    )
    desired_acceleration = np.array([0.2, -0.1, 0.05])
    beta_ddot = np.array([-0.1, 0.08, -0.04])
    disturbance_hat = np.array([0.3, -0.2, 0.1])
    actual = position_control_force(
        config, errors, desired_acceleration, beta_ddot, disturbance_hat
    )
    expected = config.total_mass_kg * (
        np.array([0.0, 0.0, GRAVITY_M_S2])
        + disturbance_hat
        - desired_acceleration
        - beta_ddot
        + config.lambda_ * errors.z_dot_ned_m_s
        + config.k * errors.sliding_ned_m_s
    )
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1.0e-14)


def test_m2_ctrl_pos_03_hover_and_mass_scaling():
    errors = position_error_terms(
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.ones(3),
    )
    base = PositionControlConfig(7.72, np.ones(3), np.ones(3))
    doubled = PositionControlConfig(15.44, np.ones(3), np.ones(3))
    zero = np.zeros(3)
    force = position_control_force(base, errors, zero, zero, zero)
    force_doubled = position_control_force(doubled, errors, zero, zero, zero)
    np.testing.assert_allclose(
        force, [0.0, 0.0, 7.72 * GRAVITY_M_S2], rtol=0.0, atol=1.0e-14
    )
    np.testing.assert_allclose(force_doubled, 2.0 * force, rtol=0.0, atol=1.0e-14)


def test_m2_ctrl_pos_04_closed_loop_sliding_identity():
    config = PositionControlConfig(7.72, [2.0, 3.0, 4.0], [3.0, 4.0, 5.0])
    errors = position_error_terms(
        [0.4, -0.2, -1.0],
        [0.2, -0.1, 0.05],
        [0.1, 0.0, -0.8],
        [0.02, 0.01, -0.03],
        [0.1, -0.1, -0.05],
        [0.03, -0.02, 0.01],
        config.lambda_,
    )
    p_ddot = np.array([0.1, -0.05, 0.02])
    beta_ddot = np.array([-0.04, 0.03, -0.02])
    disturbance = np.array([0.5, -0.3, 0.2])
    disturbance_hat = np.array([0.45, -0.25, 0.18])
    force = position_control_force(
        config, errors, p_ddot, beta_ddot, disturbance_hat
    )
    velocity_dot = position_known_acceleration(force, config.total_mass_kg) + disturbance
    sliding_dot = (
        velocity_dot - p_ddot - beta_ddot + config.lambda_ * errors.z_dot_ned_m_s
    )
    expected = -config.k * errors.sliding_ned_m_s + disturbance - disturbance_hat
    np.testing.assert_allclose(sliding_dot, expected, rtol=0.0, atol=2.0e-15)


def test_m2_ctrl_pos_05_closed_loop_stays_inside_envelope():
    paper = load_paper_default()
    config = position_control_config_from_paper(paper)
    position = np.array([0.3, -0.4, -1.0])
    desired_position = np.array([0.8, -0.8, -0.7])
    velocity = np.zeros(3)
    zero = np.zeros(3)
    disturbance = np.array([0.5, -0.3, 0.2])
    beta0 = position - desired_position
    dt = 0.002
    minimum_margin = np.inf
    for index in range(6000):
        time_s = index * dt
        trajectory_args = (
            time_s,
            beta0,
            zero,
            paper.position.performance_envelope.l,
            paper.position.preset_error_trajectory.c,
        )
        beta = preset_trajectory(*trajectory_args)
        beta_dot = preset_trajectory_dot(*trajectory_args)
        beta_ddot = preset_trajectory_ddot(*trajectory_args)
        errors = position_error_terms(
            position,
            velocity,
            desired_position,
            zero,
            beta,
            beta_dot,
            config.lambda_,
        )
        force = position_control_force(config, errors, zero, beta_ddot, disturbance)
        acceleration = position_known_acceleration(force, config.total_mass_kg) + disturbance
        velocity = velocity + dt * acceleration
        position = position + dt * velocity
        rho = envelope(
            time_s + dt,
            paper.position.performance_envelope.rho_0,
            paper.position.performance_envelope.rho_inf,
            paper.position.performance_envelope.l,
        )
        minimum_margin = min(
            minimum_margin, float(np.min(rho - np.abs(position - desired_position)))
        )
    assert minimum_margin > 0.0
    assert np.all(
        np.abs(position - desired_position)
        < paper.position.performance_envelope.rho_inf + 1.0e-3
    )


@pytest.mark.parametrize(
    "call,match",
    [
        (lambda: PositionControlConfig(0.0, np.ones(3), np.ones(3)), "total_mass_kg"),
        (lambda: PositionControlConfig(1.0, [1.0, 0.0, 1.0], np.ones(3)), "k"),
        (
            lambda: position_error_terms(
                np.zeros(2), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3)
            ),
            "position_ned_m",
        ),
        (
            lambda: position_known_acceleration([0.0, np.nan, 0.0], 1.0),
            "force_ned_n",
        ),
    ],
)
def test_m2_ctrl_pos_06_rejects_invalid_inputs(call, match):
    with pytest.raises(ValueError, match=match):
        call()


def test_m2_ctrl_att_01_desired_attitude_hover_yaw_and_right_handedness():
    hover = desired_attitude([0.0, 0.0, 10.0], 0.0)
    np.testing.assert_allclose(hover.rotation_ned_from_frd, np.eye(3), rtol=0.0, atol=1e-15)
    yaw = 0.4
    rotated = desired_attitude([0.0, 0.0, 10.0], yaw)
    expected = _rotation([0.0, 0.0, 1.0], yaw)
    np.testing.assert_allclose(rotated.rotation_ned_from_frd, expected, rtol=0.0, atol=1e-15)
    np.testing.assert_allclose(
        np.cross(rotated.b1_ned, rotated.b2_ned), rotated.b3_ned, rtol=0.0, atol=1e-15
    )
    assert np.linalg.det(rotated.rotation_ned_from_frd) == pytest.approx(1.0, rel=0.0, abs=1e-15)


def test_m2_ctrl_att_02_rejects_degenerate_desired_attitude():
    with pytest.raises(ValueError, match="模"):
        desired_attitude(np.zeros(3), 0.0)
    with pytest.raises(ValueError, match="退化"):
        desired_attitude([1.0, 0.0, 0.0], 0.0)


def test_m2_ctrl_att_03_known_quaternion_and_q_matrix_convention():
    angle = 0.3
    errors = attitude_error_terms(
        _rotation([0.0, 0.0, 1.0], angle),
        np.eye(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.ones(3),
    )
    assert errors.q0 == pytest.approx(np.cos(angle / 2.0), rel=0.0, abs=1e-15)
    np.testing.assert_allclose(
        errors.qv, [0.0, 0.0, np.sin(angle / 2.0)], rtol=0.0, atol=1e-15
    )
    expected_q = errors.q0 * np.eye(3) + skew_matrix(errors.qv)
    np.testing.assert_array_equal(errors.q_matrix, expected_q)
    assert errors.q0 >= 0.0


def test_m2_ctrl_att_04_rejects_rotation_and_180_degree_singularities():
    with pytest.raises(ValueError, match="正交"):
        attitude_error_terms(
            np.diag([1.0, 1.0, 2.0]), np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3)
        )
    with pytest.raises(ValueError, match="180"):
        attitude_error_terms(
            _rotation([1.0, 0.0, 0.0], np.pi), np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3)
        )


def test_m2_ctrl_att_05_quaternion_derivative_matches_so3_centered_difference():
    rotation = _rotation([1.0, -2.0, 0.5], 0.4)
    omega = np.array([0.3, -0.2, 0.1])
    zero = np.zeros(3)
    actual = attitude_error_terms(
        rotation, np.eye(3), omega, zero, zero, zero, np.ones(3)
    )
    step = 1.0e-6
    plus = attitude_error_terms(
        rotation @ _exp_so3(omega * step), np.eye(3), omega, zero, zero, zero, np.ones(3)
    ).qv
    minus = attitude_error_terms(
        rotation @ _exp_so3(-omega * step), np.eye(3), omega, zero, zero, zero, np.ones(3)
    ).qv
    numerical = (plus - minus) / (2.0 * step)
    np.testing.assert_allclose(actual.qv_dot, numerical, rtol=1.0e-8, atol=1.0e-11)


def test_m2_ctrl_att_06_error_and_sliding_terms_match_independent_definitions():
    rotation = _rotation([1.0, 2.0, -1.0], 0.4)
    desired = _rotation([0.0, 0.0, 1.0], 0.2)
    omega = np.array([0.3, -0.2, 0.15])
    omega_d = np.array([0.04, -0.03, 0.08])
    beta = np.array([0.08, -0.04, 0.03])
    beta_dot = np.array([-0.03, 0.02, -0.01])
    lambda_ = np.array([14.0, 13.0, 10.0])
    errors = attitude_error_terms(
        rotation, desired, omega, omega_d, beta, beta_dot, lambda_
    )
    expected_rotation_error = desired.T @ rotation
    expected_omega_tilde = omega - expected_rotation_error @ omega_d
    np.testing.assert_allclose(errors.omega_tilde, expected_omega_tilde, rtol=0.0, atol=1e-15)
    np.testing.assert_allclose(errors.z, errors.qv - beta, rtol=0.0, atol=1e-15)
    np.testing.assert_allclose(errors.z_dot, errors.qv_dot - beta_dot, rtol=0.0, atol=1e-15)
    np.testing.assert_allclose(errors.sliding, errors.z_dot + lambda_ * errors.z, rtol=0.0, atol=1e-15)


def test_m2_ctrl_att_07_nonlinear_term_and_torque_match_independent_expansion():
    paper = load_paper_default()
    inertia = load_attitude_fixture_context().inertia_kg_m2
    rotation = _rotation([1.0, 2.0, -1.0], 0.4)
    desired = _rotation([0.0, 0.0, 1.0], 0.2)
    omega = np.array([0.3, -0.2, 0.15])
    omega_d = np.array([0.04, -0.03, 0.08])
    omega_d_dot = np.array([0.01, 0.02, -0.015])
    beta_ddot = np.array([0.02, -0.015, 0.01])
    disturbance_hat = np.array([0.12, -0.08, 0.05])
    errors = attitude_error_terms(
        rotation,
        desired,
        omega,
        omega_d,
        [0.08, -0.04, 0.03],
        [-0.03, 0.02, -0.01],
        paper.attitude.control.lambda_diagonal,
    )
    actual = attitude_control_torque(
        inertia,
        omega,
        errors,
        omega_d,
        omega_d_dot,
        beta_ddot,
        disturbance_hat,
        paper.attitude.control.lambda_diagonal,
        paper.attitude.control.k_diagonal,
    )
    gyroscopic = np.linalg.solve(inertia, np.cross(omega, inertia @ omega))
    nonlinear = (
        0.5 * errors.q_matrix_dot @ errors.omega_tilde
        - 0.5 * errors.q_matrix @ gyroscopic
        - 0.5 * errors.q_matrix @ errors.rotation_error_dot @ omega_d
        - 0.5 * errors.q_matrix @ errors.rotation_error @ omega_d_dot
    )
    bracket = (
        -nonlinear
        - 0.5 * errors.q_matrix @ disturbance_hat
        + beta_ddot
        - paper.attitude.control.lambda_diagonal * errors.z_dot
        - paper.attitude.control.k_diagonal * errors.sliding
    )
    torque = 2.0 * inertia @ np.linalg.solve(errors.q_matrix, bracket)
    np.testing.assert_allclose(actual.nonlinear_term_frd_rad_s2, nonlinear, rtol=1e-13, atol=1e-14)
    np.testing.assert_allclose(actual.torque_frd_n_m, torque, rtol=1e-13, atol=1e-14)


def test_m2_ctrl_att_08_closed_loop_sliding_identity():
    paper = load_paper_default()
    inertia = load_attitude_fixture_context().inertia_kg_m2
    omega = np.array([0.2, -0.1, 0.15])
    omega_d = np.array([0.03, -0.02, 0.04])
    omega_d_dot = np.array([0.01, -0.005, 0.008])
    beta_ddot = np.array([0.02, -0.01, 0.015])
    disturbance = np.array([0.3, -0.2, 0.1])
    disturbance_hat = np.array([0.27, -0.18, 0.08])
    errors = attitude_error_terms(
        _rotation([1.0, 1.0, -0.5], 0.35),
        _rotation([0.0, 0.0, 1.0], 0.1),
        omega,
        omega_d,
        [0.04, -0.03, 0.02],
        [-0.01, 0.015, -0.005],
        paper.attitude.control.lambda_diagonal,
    )
    control = attitude_control_torque(
        inertia,
        omega,
        errors,
        omega_d,
        omega_d_dot,
        beta_ddot,
        disturbance_hat,
        paper.attitude.control.lambda_diagonal,
        paper.attitude.control.k_diagonal,
    )
    qv_ddot = (
        control.nonlinear_term_frd_rad_s2
        + 0.5 * errors.q_matrix @ np.linalg.solve(inertia, control.torque_frd_n_m)
        + 0.5 * errors.q_matrix @ disturbance
    )
    sliding_dot = (
        qv_ddot
        - beta_ddot
        + paper.attitude.control.lambda_diagonal * errors.z_dot
    )
    expected = (
        -paper.attitude.control.k_diagonal * errors.sliding
        + 0.5 * errors.q_matrix @ (disturbance - disturbance_hat)
    )
    np.testing.assert_allclose(sliding_dot, expected, rtol=1e-13, atol=2e-14)


def test_m2_ctrl_att_09_closed_loop_stays_inside_envelope():
    paper = load_paper_default()
    inertia = load_attitude_fixture_context().inertia_kg_m2
    angle = 0.3
    rotation = _rotation([1.0, 0.0, 0.0], angle)
    desired_rotation = np.eye(3)
    omega = np.zeros(3)
    zero = np.zeros(3)
    disturbance = np.array([0.2, -0.1, 0.15])
    beta0 = attitude_error_terms(
        rotation,
        desired_rotation,
        omega,
        zero,
        zero,
        zero,
        paper.attitude.control.lambda_diagonal,
    ).qv
    dt = 0.001
    minimum_margin = np.inf
    qv = beta0
    for index in range(8000):
        time_s = index * dt
        trajectory_args = (
            time_s,
            beta0,
            zero,
            paper.attitude.performance_envelope.l,
            paper.attitude.preset_error_trajectory.c,
        )
        beta = preset_trajectory(*trajectory_args)
        beta_dot = preset_trajectory_dot(*trajectory_args)
        beta_ddot = preset_trajectory_ddot(*trajectory_args)
        errors = attitude_error_terms(
            rotation,
            desired_rotation,
            omega,
            zero,
            beta,
            beta_dot,
            paper.attitude.control.lambda_diagonal,
        )
        control = attitude_control_torque(
            inertia,
            omega,
            errors,
            zero,
            zero,
            beta_ddot,
            disturbance,
            paper.attitude.control.lambda_diagonal,
            paper.attitude.control.k_diagonal,
        )
        omega_dot = (
            np.linalg.solve(
                inertia,
                control.torque_frd_n_m - np.cross(omega, inertia @ omega),
            )
            + disturbance
        )
        omega = omega + dt * omega_dot
        rotation = rotation @ _exp_so3(omega * dt)
        qv = attitude_error_terms(
            rotation,
            desired_rotation,
            omega,
            zero,
            zero,
            zero,
            paper.attitude.control.lambda_diagonal,
        ).qv
        rho = envelope(
            time_s + dt,
            paper.attitude.performance_envelope.rho_0,
            paper.attitude.performance_envelope.rho_inf,
            paper.attitude.performance_envelope.l,
        )
        minimum_margin = min(minimum_margin, float(np.min(rho - np.abs(qv))))
    assert minimum_margin > 0.0
    assert np.all(
        np.abs(qv) < paper.attitude.performance_envelope.rho_inf + 1.0e-3
    )


def test_m2_ctrl_att_10_validation_ownership_and_read_only_outputs():
    rotation = np.eye(3)
    omega = np.array([0.1, -0.2, 0.3])
    omega_before = omega.copy()
    errors = attitude_error_terms(
        rotation, rotation, omega, np.zeros(3), np.zeros(3), np.zeros(3), np.ones(3)
    )
    np.testing.assert_array_equal(omega, omega_before)
    assert errors.qv.flags.writeable is False
    assert errors.q_matrix.flags.writeable is False
    with pytest.raises(ValueError, match="正定"):
        attitude_control_torque(
            np.diag([1.0, 1.0, 0.0]),
            omega,
            errors,
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            np.ones(3),
            np.ones(3),
        )
    with pytest.raises(ValueError, match="k"):
        attitude_control_torque(
            np.eye(3),
            omega,
            errors,
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            np.ones(3),
            [1.0, 0.0, 1.0],
        )
