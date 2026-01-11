# SensorReading.msg
# 센서 ID와 타임스탬프를 포함하는 기본 헤더 정보
uint32 sensor_id
builtin_interfaces/Time timestamp

# 센서 위치 정보 (3D 좌표)
geometry_msgs/Point position

# 센서 측정값들 (온도, 습도, 압력)
float64 temperature
float64 humidity
float64 pressure

# 센서 상태 플래그
bool is_active
bool has_error

# 추가 메타데이터
string sensor_name
string location