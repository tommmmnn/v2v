
std_msgs/Header header
float32 speed
float32 roadWheelAngle

#steerMotor
float32 motorSpeed
bool steerMotorEnabled
uint8 steerMotorError

uint8 MOTOR_ERROR_NON = 0
uint8 MOTOR_ERROR_OVER_CURRENT = 1
uint8 MOTOR_ERROR_OVER_VOLTAGE = 2
uint8 MOTOR_ERROR_UNDER_VOLTAGE = 3
uint8 MOTOR_ERROR_OVER_LOAD = 15
uint8 MOTOR_ERROR_LOCKED_ROTOR = 17
uint8 MOTOR_ERROR_OVER_SPEED = 20
uint8 MOTOR_ERROR_POWER_BAN = 26

uint8 MOTOR_ANGLE_SENSOR_ERROR = 254
uint8 MOTOR_ERROR_OFFLINE = 255


#brakeSystem
uint8 brake_val #0-100
uint8 brakeError

uint8 BRAKE_ERROR_NONE = 0
uint8 BRAKE_ERROR_OFFLINE = 1
uint8 BRAKE_ERROR_ABNORMAL = 2



