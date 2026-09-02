/**
 * @file hk_camera_node_main.cpp
 * @brief 纯 ROS2 节点入口 (无 GUI/headless 模式)
 *
 * 适用于:
 *   - 不需要 Qt 界面的部署场景
 *   - 作为 ROS2 组件节点运行
 *
 * 启动方式:
 *   ros2 run hk_camera hk_camera_node
 */

#include <rclcpp/rclcpp.hpp>
#include "hk_camera/hk_camera_node.h"
#include "hk_camera/hk_camera.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // 初始化 HK SDK
    auto* camera = hk_camera::HKCamera::instance();
    camera->init();

    // 创建并启动 ROS2 节点
    auto node = std::make_shared<hk_camera::HKCameraNode>("hk_camera_node");

    RCLCPP_INFO(node->get_logger(), "HKCamera headless node starting...");

    rclcpp::spin(node);

    // 清理
    camera->cleanup();
    rclcpp::shutdown();

    return 0;
}
