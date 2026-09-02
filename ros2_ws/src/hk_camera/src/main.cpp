/**
 * @file main.cpp
 * @brief 海康相机 Qt + ROS2 应用程序入口
 *
 * 架构:
 *   1. Qt Application (主线程, 处理 GUI 事件循环)
 *   2. ROS2 Node (单独线程, 处理 pub/sub/service)
 *   3. HKCamera (Qt 单例, 桥接 SDK 回调 → Qt signals)
 *
 * 启动方式:
 *   ros2 run hk_camera hk_camera_gui
 *   ros2 launch hk_camera hk_camera.launch.py  (待添加)
 *
 * 线程模型:
 *   - 主线程:    Qt GUI (MainWindow + 事件循环)
 *   - 线程1:     ROS2 spinning (HKCameraNode)
 *   - 线程N:     海康 SDK 内部回调线程 (由 libhcnetsdk.so 管理)
 */

#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <memory>
#include <signal.h>

#include "hk_camera/hk_camera_node.h"
#include "main_window.h"

using namespace hk_camera;

// 全局变量用于信号处理
static std::atomic<bool> g_running{true};
static std::shared_ptr<HKCameraNode> g_node;
static std::unique_ptr<std::thread> g_ros_thread;

void sigintHandler(int)
{
    g_running = false;
    if (g_node)
    {
        rclcpp::shutdown();
    }
    QApplication::quit();
}

/**
 * @brief ROS2 spin 线程
 */
void rosSpinThread()
{
    rclcpp::spin(g_node);
}

int main(int argc, char* argv[])
{
    // ==================== 1. Qt 初始化 ====================
    QApplication app(argc, argv);
    app.setApplicationName("hk_camera_gui");
    app.setApplicationVersion("0.1.0");

    // ==================== 2. ROS2 初始化 ====================
    rclcpp::init(argc, argv);

    // 可选的 ROS2 参数: 若只想用 Qt 模式不带 ROS2, 传递 --no-ros
    bool enable_ros = true;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "--no-ros")
        {
            enable_ros = false;
            break;
        }
    }

    // ==================== 3. 创建 ROS2 节点 (在线程中运行) ====================
    if (enable_ros)
    {
        g_node = std::make_shared<HKCameraNode>("hk_camera_node");
        g_ros_thread = std::make_unique<std::thread>(rosSpinThread);
    }

    // ==================== 4. 创建 Qt 主窗口 ====================
    MainWindow window;
    if (enable_ros && g_node)
    {
        window.setRosNode(g_node);
    }
    window.show();

    // ==================== 5. 信号处理 ====================
    signal(SIGINT,  sigintHandler);
    signal(SIGTERM, sigintHandler);

    // ==================== 6. Qt 事件循环 ====================
    int ret = app.exec();

    // ==================== 7. 清理 ====================
    g_running = false;
    if (g_ros_thread && g_ros_thread->joinable())
    {
        rclcpp::shutdown();
        g_ros_thread->join();
    }
    rclcpp::shutdown();

    return ret;
}
