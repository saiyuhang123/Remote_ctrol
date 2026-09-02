# 海康相机报警信息
uint32 alarm_type           # 报警类型 (3=移动侦测)
int32[] alarm_channels      # 触发报警的通道列表
string alarm_desc           # 报警描述
builtin_interfaces/Time stamp  # 报警时间戳
