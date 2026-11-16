#!/usr/bin/env python3
"""pregme_control 节点的 launch 级安全测试。

在真实 ROS 2 运行时中启动节点，测试进程扮演 PX4 和地面端：
  - 向八个 /fmu/out/* 话题注入健康输入；
  - 监听三个 /fmu/in/* 控制话题，统计节点是否越权发布控制；
  - 通过参数服务切换模式、尝试修改启动期参数；
  - 断言「算法未就绪时，任何模式都必须保持输出静默」以及参数白名单生效。

注意：真正让算法就绪还需要 M2/M3 的实现；这里锁死的是当前的安全边界。
"""

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
    """启动被测节点。提高状态发布频率可缩短测试等待时间。"""
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
        """建立测试节点、输出监听器、参数客户端和八个输入发布器。"""
        rclpy.init()
        cls.node = rclpy.create_node("pregme_control_safety_test")
        # 计数节点向三个 /fmu/in 话题发布控制消息的次数；期望始终为 0。
        cls.output_counts = {"mode": 0, "thrust": 0, "torque": 0}
        # 缓存最新一条 PregmeStatus，供断言读取。
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
        # 参数客户端用于运行期切换模式和验证启动期参数会被拒绝。
        cls.parameter_client = cls.node.create_client(
            SetParameters, "/pregme_control/set_parameters"
        )
        # 模拟 PX4 的八个输入话题；键名与节点参数中的通道名对应。
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
        # 单调递增的采样时间戳，满足节点的时间戳单调性要求。
        cls.sample_timestamp_us = 1_000_000

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # -------------------------------------------------------------------------
    # 回调与辅助函数
    # -------------------------------------------------------------------------
    @classmethod
    def _increment(cls, output_name):
        """输出话题回调：累计节点发布控制消息的次数。"""
        cls.output_counts[output_name] += 1

    @classmethod
    def _store_status(cls, message):
        """状态话题回调：保存最新状态消息。"""
        cls.latest_status = message

    @classmethod
    def _spin_until(cls, predicate, timeout_s=5.0):
        """以小步长 spin，直到谓词为真或超时。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def _publish_valid_inputs(cls):
        """注入一组全部健康、时间戳统一递增的 PX4 输入。

        每个字段都按节点的 valid 判据填写：位置/速度四个有效位为真、
        四元数为单位四元数、已解锁且 Offboard、分配器已达成且无电机故障、
        参考轨迹有限、原生推力/力矩有限。
        """
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
        """在指定时长内持续注入健康输入，避免节点超时进入故障状态。"""
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            cls._publish_valid_inputs()
            rclpy.spin_once(cls.node, timeout_sec=0.02)

    @classmethod
    def _set_mode_and_enable(cls, mode):
        """一次性设置 mode 并打开 enable_control_output，返回服务响应。"""
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
        """核心算法未就绪时，模式 0~3 都不能向 PX4 发布任何控制指令。

        即使健康输入齐全、输出开关已打开、模式被切到 3，节点也必须保持
        三个 /fmu/in 话题零发布；同时各通道有效位应全部置位（255）。
        """
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
        """运行期修改启动期参数（如 state_timeout_ms）必须被参数回调拒绝。"""
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
