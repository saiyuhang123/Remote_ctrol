from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hi_ros2",
            executable="hi_ros2_node",
            name="hi_ros2_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"frame_name": "laser_fe",
                 "topic_name": "scan_fe",
                 "service_name": "HinsfeSrv",

                 "laser_type": 2,                   # 1:he   2:fe 3: se  4: le 设置连接传感器系列；      
                        
                 "laser_ip": "192.168.1.88",		# 传感器ip地址		
                 "port": 8080,				        # 传感器端口（固定为8080）
                 "use_udp": False,                   # True：UDP链接方式；False：TCP链接方式；
                 "synctype": False,                 # True：读取NTP格式报文（传感器版本号大于2.0.0）；False：获取标准报文；

                 "block_enable": False,             # True：获取障碍物检测结果；False：不获取障碍物检测；
                 "change_param": True,              # 是否改变雷达运行参数（False：不改变：True：改变）;
                 "angle_increment": "0.100",        # 设置运行分辨率（0.025；0.050；0.100；0.250；0.500);
                 "spin_frequency_Hz": 25,           # 设置扫描频率（12:12.5hz；25；25hz；50：50hz）;
                 "noise_filter_level": 1,           # 噪声过滤等级（0：关闭;1:简单；2：中等；3：严格；） ;
                 
                 "start_angle": -30.0,                # 设置输出角度最小值-180.0--180.0，过滤角度值强制等于1024；
                 "end_angle": 30.0,                # 设置输出角度最大值-180.0--180.0，过滤角度值强制等于1024；
                 "offset_angle": 0.0,               # 坐标旋转角度设置，-180---+180.0 传感器无出线位置为正前方0°；

                 "shadows_filter_level": 0,			# 防拖尾过滤等级(-1：自定义；0：关闭；1-3：过滤等级，等级越高但是更消耗资源；)
                 "shadows_filter_max_angle": 175.0,	# 筛选的夹角最大值
                 "shadows_filter_min_angle": 5.0,	# 筛选的夹角最小值
                 "shadows_filter_neighbors": 1,		# 触发过滤所需的最少邻居点数
                 "shadows_filter_window": 5,		# 搜索窗口大小(检测遍历点的后n个值)
                 "shadows_filter_traverse_step": 1,	# 遍历点数的步长(每n个点检测一次)


                 }				
            ]
        )
    ])
