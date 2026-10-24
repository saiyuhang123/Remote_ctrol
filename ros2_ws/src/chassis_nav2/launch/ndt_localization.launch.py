from launch import LaunchDescription
from launch.actions import OpaqueFunction
from chassis_nav2.ndt_bringup import arguments, configure


def generate_launch_description():
    return LaunchDescription(arguments()+[OpaqueFunction(function=configure)])
