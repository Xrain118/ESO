#!/usr/bin/env python3

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from eso_interfaces.msg import PregmeStatus
from px4_msgs.msg import ControlAllocatorStatus
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleLocalPositionSetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters


def generate_test_description():
    control_node = launch_ros.actions.Node(
        package="pregme_control",
        executable="pregme_control_node",
        output="screen",
        parameters=[{"status_rate_hz": 100.0}],
    )
    return (
        launch.LaunchDescription(
            [
                control_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"control_node": control_node},
    )


class TestNodeSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("pregme_control_safety_test")
        cls.output_counts = {"mode": 0, "thrust": 0, "torque": 0}
        cls.latest_status = None
        cls.subscriptions = [
            cls.node.create_subscription(
                OffboardControlMode,
                "/fmu/in/offboard_control_mode",
                lambda _: cls._increment("mode"),
                10,
            ),
            cls.node.create_subscription(
                VehicleThrustSetpoint,
                "/fmu/in/vehicle_thrust_setpoint",
                lambda _: cls._increment("thrust"),
                10,
            ),
            cls.node.create_subscription(
                VehicleTorqueSetpoint,
                "/fmu/in/vehicle_torque_setpoint",
                lambda _: cls._increment("torque"),
                10,
            ),
            cls.node.create_subscription(
                PregmeStatus,
                "/eso/pregme/status",
                cls._store_status,
                10,
            ),
        ]
        cls.parameter_client = cls.node.create_client(
            SetParameters, "/pregme_control/set_parameters"
        )
        cls.input_publishers = {
            "local": cls.node.create_publisher(
                VehicleLocalPosition, "/fmu/out/vehicle_local_position", 10
            ),
            "attitude": cls.node.create_publisher(
                VehicleAttitude, "/fmu/out/vehicle_attitude", 10
            ),
            "angular_velocity": cls.node.create_publisher(
                VehicleAngularVelocity,
                "/fmu/out/vehicle_angular_velocity",
                10,
            ),
            "vehicle_status": cls.node.create_publisher(
                VehicleStatus, "/fmu/out/vehicle_status_v1", 10
            ),
            "allocator": cls.node.create_publisher(
                ControlAllocatorStatus,
                "/fmu/out/control_allocator_status",
                10,
            ),
            "reference": cls.node.create_publisher(
                VehicleLocalPositionSetpoint,
                "/fmu/out/vehicle_local_position_setpoint",
                10,
            ),
            "native_thrust": cls.node.create_publisher(
                VehicleThrustSetpoint,
                "/fmu/out/vehicle_thrust_setpoint",
                10,
            ),
            "native_torque": cls.node.create_publisher(
                VehicleTorqueSetpoint,
                "/fmu/out/vehicle_torque_setpoint",
                10,
            ),
        }
        cls.sample_timestamp_us = 1_000_000

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _increment(cls, output_name):
        cls.output_counts[output_name] += 1

    @classmethod
    def _store_status(cls, message):
        cls.latest_status = message

    @classmethod
    def _spin_until(cls, predicate, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def _publish_valid_inputs(cls):
        cls.sample_timestamp_us += 10_000
        timestamp = cls.sample_timestamp_us

        local = VehicleLocalPosition()
        local.timestamp_sample = timestamp
        local.xy_valid = True
        local.z_valid = True
        local.v_xy_valid = True
        local.v_z_valid = True
        cls.input_publishers["local"].publish(local)

        attitude = VehicleAttitude()
        attitude.timestamp_sample = timestamp
        attitude.q = [1.0, 0.0, 0.0, 0.0]
        cls.input_publishers["attitude"].publish(attitude)

        angular_velocity = VehicleAngularVelocity()
        angular_velocity.timestamp_sample = timestamp
        angular_velocity.xyz = [0.0, 0.0, 0.0]
        cls.input_publishers["angular_velocity"].publish(angular_velocity)

        vehicle_status = VehicleStatus()
        vehicle_status.timestamp = timestamp
        vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        vehicle_status.failsafe = False
        cls.input_publishers["vehicle_status"].publish(vehicle_status)

        allocator = ControlAllocatorStatus()
        allocator.timestamp = timestamp
        allocator.torque_setpoint_achieved = True
        allocator.thrust_setpoint_achieved = True
        allocator.unallocated_torque = [0.0, 0.0, 0.0]
        allocator.unallocated_thrust = [0.0, 0.0, 0.0]
        allocator.handled_motor_failure_mask = 0
        cls.input_publishers["allocator"].publish(allocator)

        reference = VehicleLocalPositionSetpoint()
        reference.timestamp = timestamp
        reference.x = 0.0
        reference.y = 0.0
        reference.z = -1.0
        reference.vx = 0.0
        reference.vy = 0.0
        reference.vz = 0.0
        reference.acceleration = [0.0, 0.0, 0.0]
        reference.yaw = 0.0
        reference.yawspeed = 0.0
        cls.input_publishers["reference"].publish(reference)

        native_thrust = VehicleThrustSetpoint()
        native_thrust.timestamp = timestamp
        native_thrust.timestamp_sample = timestamp
        native_thrust.xyz = [0.0, 0.0, -0.5]
        cls.input_publishers["native_thrust"].publish(native_thrust)

        native_torque = VehicleTorqueSetpoint()
        native_torque.timestamp = timestamp
        native_torque.timestamp_sample = timestamp
        native_torque.xyz = [0.0, 0.0, 0.0]
        cls.input_publishers["native_torque"].publish(native_torque)

    @classmethod
    def _spin_with_inputs(cls, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            cls._publish_valid_inputs()
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def _set_mode_and_enable(cls, mode):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="mode",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=mode,
                ),
            ),
            Parameter(
                name="enable_control_output",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_BOOL,
                    bool_value=True,
                ),
            ),
        ]
        future = cls.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=3.0)
        return future.result()

    def test_all_modes_remain_output_silent_without_algorithm(self):
        self.assertTrue(
            self._spin_until(lambda: self.latest_status is not None),
            "未收到节点状态主题",
        )
        self.assertTrue(
            self.parameter_client.wait_for_service(timeout_sec=3.0),
            "参数服务不可用",
        )

        for mode in range(4):
            self._spin_with_inputs(0.25)
            response = self._set_mode_and_enable(mode)
            self.assertIsNotNone(response)
            self.assertTrue(all(result.successful for result in response.results))
            self.assertTrue(
                self._spin_until(
                    lambda mode=mode: self.latest_status.requested_mode == mode
                )
            )
            self.assertFalse(self.latest_status.algorithm_ready)
            self._spin_with_inputs(0.15)

        self.assertEqual(self.latest_status.validity_flags, 255)

        self.assertEqual(
            self.output_counts,
            {"mode": 0, "thrust": 0, "torque": 0},
        )

    def test_startup_only_parameter_is_rejected(self):
        self.assertTrue(self.parameter_client.wait_for_service(timeout_sec=3.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="state_timeout_ms",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=300,
                ),
            )
        ]
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        response = future.result()
        self.assertIsNotNone(response)
        self.assertEqual(len(response.results), 1)
        self.assertFalse(response.results[0].successful)
