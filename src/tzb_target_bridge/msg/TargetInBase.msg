std_msgs/Header header

# 物体在底盘坐标系 base_frame 下的位置（米）
float64 x
float64 y
float64 z

# 物体朝向（绕 z 轴，弧度）
float64 yaw_rad

# 派生量：到车体原点的水平距离、方位角（+y 为左，弧度）
float64 distance_m
float64 bearing_rad
