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
                {"frame_name": "laser_de",
                 "topic_name": "scan_de",
                 "service_name": "HinsDeSrv",                 

                 "laser_type": 5,                   # 设置连接传感器系列； 1:he   2:fe 3: se  4: le
                 "laser_ip": "192.168.1.88",	    # 传感器ip地址		
                 "port": 8080,				        # 传感器端口（固定为8080）
                 "use_udp": True,                  # True：UDP链接方式；False：TCP链接方式；                  

                 "block_enable": False,              # True：获取障碍物检测结果；False：不获取障碍物检测；   

                 "start_angle": -60.0,               # 设置输出角度最小值，过滤角度值强制等于1024；
                 "end_angle": 60.0,                # 设置输出角度最大值，过滤角度值强制等于1024；
                 "offset_angle": 0.0,               # 坐标旋转角度设置，0.0--360.0；

                 "shadows_filter_level": 0,         # 防拖尾过滤等级(-1：自定义；0：关闭；1-3：过滤等级，等级越高但是更消耗资源；)
                 "shadows_filter_max_angle": 175.0, # 筛选的夹角最大值
                 "shadows_filter_min_angle": 5.0,   # 筛选的夹角最小值
                 "shadows_filter_neighbors": 1,     # 触发过滤所需的最少邻居点数
                 "shadows_filter_window": 5,        # 搜索窗口大小(检测遍历点的后n个值)
                 "shadows_filter_traverse_step": 1, # 遍历点数的步长(每n个点检测一次) 
                 }				
            ]
        )
    ])
