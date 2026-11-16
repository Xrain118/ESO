uint8 LEVEL_INFO = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_ERROR = 2

uint8 SOURCE_GATE = 0
uint8 SOURCE_LOCAL_POSITION = 1
uint8 SOURCE_ATTITUDE = 2
uint8 SOURCE_ANGULAR_VELOCITY = 3
uint8 SOURCE_VEHICLE_STATUS = 4
uint8 SOURCE_ALLOCATOR_STATUS = 5
uint8 SOURCE_REFERENCE = 6
uint8 SOURCE_NATIVE_THRUST = 7
uint8 SOURCE_NATIVE_TORQUE = 8
uint8 SOURCE_PARAMETERS = 9
uint8 SOURCE_SIMULATION_CLOCK = 10

uint64 timestamp
uint64 timestamp_sample
uint8 level
uint8 source
uint8 requested_mode
uint8 active_mode
uint32 fault_flags
uint32 reset_reason_flags
uint32 reset_count
uint64 event_sequence
string reason
