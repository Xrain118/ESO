uint8 MODE_NATIVE = 0
uint8 MODE_SHADOW = 1
uint8 MODE_BOUNDED_COMPENSATION = 2
uint8 MODE_FULL_CONTROL = 3

uint32 VALID_LOCAL_POSITION = 1
uint32 VALID_ATTITUDE = 2
uint32 VALID_ANGULAR_VELOCITY = 4
uint32 VALID_VEHICLE_STATUS = 8
uint32 VALID_ALLOCATOR_STATUS = 16
uint32 VALID_REFERENCE = 32
uint32 VALID_NATIVE_THRUST = 64
uint32 VALID_NATIVE_TORQUE = 128

uint32 FAULT_INVALID_MODE = 1
uint32 FAULT_LOCAL_POSITION = 2
uint32 FAULT_ATTITUDE = 4
uint32 FAULT_ANGULAR_VELOCITY = 8
uint32 FAULT_VEHICLE_STATUS = 16
uint32 FAULT_ALLOCATOR_STATUS = 32
uint32 FAULT_TIMESTAMP = 64
uint32 FAULT_ESTIMATOR_RESET = 128
uint32 FAULT_NON_FINITE = 256
uint32 FAULT_ALGORITHM_UNAVAILABLE = 512
uint32 FAULT_OUTPUT_DISABLED = 1024
uint32 FAULT_REFERENCE = 2048
uint32 FAULT_NATIVE_THRUST = 4096
uint32 FAULT_NATIVE_TORQUE = 8192
uint32 FAULT_PARAMETER_INVALID = 16384
uint32 FAULT_PLACEHOLDER_PARAMETERS = 32768
uint32 FAULT_SIMULATION_CONTEXT = 65536
uint32 FAULT_NOT_ARMED = 131072
uint32 FAULT_NOT_OFFBOARD = 262144
uint32 FAULT_FAILSAFE = 524288
uint32 FAULT_REENTRY_REQUIRED = 1048576

uint64 timestamp
uint64 timestamp_sample
uint64 local_position_timestamp_sample
uint64 attitude_timestamp_sample
uint64 angular_velocity_timestamp_sample
uint64 vehicle_status_timestamp
uint64 allocator_timestamp
uint64 reference_timestamp
uint64 native_thrust_timestamp_sample
uint64 native_torque_timestamp_sample
uint8 requested_mode
uint8 active_mode
bool algorithm_ready
bool input_valid
bool output_enabled
bool has_placeholder
bool simulation_context_valid
uint32 validity_flags
uint32 fault_flags
uint32 reset_count
uint32 reset_reason_flags
string reason
string fixture_id
string parameter_set_sha256
string fixture_watermark
float32[3] position_ned
float32[3] velocity_ned
float32[4] attitude_wxyz
float32[3] angular_velocity_frd
float32[3] reference_position_ned
float32[3] reference_velocity_ned
float32[3] reference_acceleration_ned
float32 reference_yaw_ned
float32 reference_yawspeed_ned
float32[3] native_thrust_setpoint_frd
float32[3] native_torque_setpoint_frd
float32[3] thrust_setpoint_frd
float32[3] torque_setpoint_frd
