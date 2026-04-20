# 外部控制指令消息
# 支持位置、速度、加速度控制模式

std_msgs/Header header

uint8 control_type
uint8 POSITION=0
uint8 VELOCITY=1
uint8 ACCELERATION=2
uint8 EMERGENCY_LAND=99

geometry_msgs/Pose pose        # 位置/姿态设定点（POSITION模式）
geometry_msgs/Twist twist      # 速度设定点（VELOCITY模式）
geometry_msgs/Accel accel      # 加速度设定点（ACCELERATION模式）

float64[4] covariance          # 指令不确定性 [x, y, z, yaw]

uint8 priority                 # 指令优先级（0-255，高优先级覆盖）
bool is_relative               # 是否相对当前位置
string source_id               # 指令源标识符