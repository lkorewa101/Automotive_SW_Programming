# RobotStatus.msg
# 로봇 상태 상수 정의
uint8 STATUS_IDLE = 0
uint8 STATUS_MOVING = 1
uint8 STATUS_ERROR = 2
uint8 STATUS_CHARGING = 3

# 헤더 정보
std_msgs/Header header

# 기본 상태 정보
uint8 status
float64 battery_level
float64 speed

# 센서 데이터 배열
SensorReading[] sensor_readings

# 위치 및 방향 정보
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity

# 오류 정보
bool[] error_flags
string[] error_messages