Header header

int8 MOTOR_ID_FRONT = 0
int8 MOTOR_ID_REAR_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2

# motion state
float64 linear_velocity
float64 steering_angle

# base state
uint8 base_state
uint8 control_mode
uint16 fault_code
float64 battery_voltage
uint8 park_mode

# motor state
HunterMotorState[3] motor_states
# driver state
HunterDriverState[3] driver_states
