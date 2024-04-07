#uint8 GT_GT=1 # 真值框（检出）
#uint8 FN_GT=2 # 真值框（漏检）
#uint8 TP_PRED=3 # 预测框（真阳）
#uint8 FP_PRED=4 # 预测框（假阳）

uint8 GT=1 # 真值框（检出）
uint8 FN=2 # 真值框（漏检）
uint8 TP=3 # 预测框（真阳）
uint8 FP=4 # 预测框（假阳）

# BoundingBox represents a oriented bounding box.
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions # x, y, z 分别对应长、宽、高
float32 value # 得分
uint32 label # 类别信息
uint32 point_num # 激光点个数
uint8 attr # 框的属性