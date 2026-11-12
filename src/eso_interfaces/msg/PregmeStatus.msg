uint8 MODE_NATIVE = 0
uint8 MODE_SHADOW = 1
uint8 MODE_BOUNDED_COMPENSATION = 2
uint8 MODE_FULL_CONTROL = 3

uint32 VALID_LOCAL_POSITION = 1
uint32 VALID_ATTITUDE = 2
uint32 VALID_ANGULAR_VELOCITY = 4
uint32 VALID_VEHICLE_STATUS = 8
uint32 VALID_ALLOCATOR_STATUS = 16

uint64 timestamp
uint64 timestamp_sample
uint8 requested_mode
uint8 active_mode
bool algorithm_ready
bool input_valid
bool output_enabled
uint32 validity_flags
uint32 fault_flags
uint32 reset_count
string reason
float32[3] position_ned
float32[3] velocity_ned
float32[4] attitude_wxyz
float32[3] angular_velocity_frd
float32[3] thrust_setpoint_frd
float32[3] torque_setpoint_frd
