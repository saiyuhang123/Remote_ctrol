# 任务点位: 导航位姿 + 云台参数 + 动作
geometry_msgs/PoseStamped nav_pose   # 导航目标位姿
float32 pan                          # 云台预置点编号 (1~256, 海康SDK用预置点定位)
float32 tilt                         # 预留: 垂直参数
float32 zoom                         # 预留: 变倍参数
bool do_capture                      # 到达后是否拍照
string extra_action                  # 预留扩展动作名
