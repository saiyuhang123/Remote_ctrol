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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <cstdlib>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include "hk_camera/hk_camera_node.h"
#include "hk_camera/hk_camera.h"

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (!std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE")) {
        QString profile = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(
            "../../share/hk_camera/config/camera_dds_profiles.xml");
        if (!QFileInfo::exists(profile)) profile = QString::fromUtf8(HK_DDS_PROFILE);
        if (!QFileInfo::exists(profile) ||
            ::setenv("FASTRTPS_DEFAULT_PROFILES_FILE", profile.toLocal8Bit().constData(), 0) != 0) return 1;
    }
    // Full-resolution images and capture replies must not perform DDS network
    // delivery on the SDK/executor thread. Scope this to the camera process;
    // an explicitly supplied middleware policy still takes precedence.
    if (::setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS", 0) != 0) return 1;
    rclcpp::init(argc, argv);

    // 初始化 HK SDK
    auto* camera = hk_camera::HKCamera::instance();
    if (!camera->init())
    {
        RCLCPP_ERROR(rclcpp::get_logger("hk_camera_node"),
                    "HK SDK initialization failed: %d", camera->getLastError());
        rclcpp::shutdown();
        return 1;
    }

    // 创建并启动 ROS2 节点
    auto node = std::make_shared<hk_camera::HKCameraNode>("hk_camera_node");

    RCLCPP_INFO(node->get_logger(), "HKCamera headless node starting...");

    // A capture can spend time preparing/sending a full-resolution response.
    // Its separate callback group must not hold up lens queries or watchdogs.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);

    // 清理
    node.reset();
    rclcpp::shutdown();

    return 0;
}
