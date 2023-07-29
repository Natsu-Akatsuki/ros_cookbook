# 定义类别常量
uint8 UNKNOWN=0
uint8 CAR=1
uint8 TRUCK=2
uint8 BUS=3
uint8 BICYCLE=4
uint8 MOTORBIKE=5
uint8 PEDESTRIAN=6
uint8 ANIMAL=7

uint32 class_id
uint32 track_id
float32 confidence

# 位置（左上和右下）
int32 top_left_x
int32 top_left_y
int32 bottom_right_x
int32 bottom_right_y
float32 depth

# 速度
float32 velocity_x
float32 velocity_y
float32 velocity_z


