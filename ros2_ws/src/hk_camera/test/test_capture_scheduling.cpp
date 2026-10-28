#include "hk_camera/hk_camera_node.h"
#include <gtest/gtest.h>
#include <QCoreApplication>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <chrono>
#include <future>
#include <thread>

using namespace std::chrono_literals;

TEST(CameraScheduling, LensQueryRunsWhileTheRealCaptureGroupIsOccupied)
{
    rclcpp::init(0, nullptr);
    struct Shutdown { ~Shutdown() { rclcpp::shutdown(); } } shutdown;
    int argc = 1;
    char name[] = "capture_scheduling_test";
    char* argv[] = {name, nullptr};
    QCoreApplication application(argc, argv);
    // auto_login defaults to false: the SDK is initialized but no device is
    // contacted. A real lens query must return its not-logged-in response.
    auto camera = std::make_shared<hk_camera::HKCameraNode>();
    auto client_node = std::make_shared<rclcpp::Node>("capture_scheduling_client");
    rclcpp::CallbackGroup::SharedPtr capture_group;
    camera->get_node_base_interface()->for_each_callback_group(
        [&](rclcpp::CallbackGroup::SharedPtr group) {
            if (group->find_service_ptrs_if([](rclcpp::ServiceBase::SharedPtr service) {
                    return std::string(service->get_service_name()) == "/hk_camera/capture";
                })) capture_group = group;
        });
    ASSERT_TRUE(capture_group);
    ASSERT_NE(capture_group, camera->get_node_base_interface()->get_default_callback_group());
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(camera);
    executor.add_node(client_node);
    std::promise<void> release;
    auto released = release.get_future().share();
    struct Spin {
        rclcpp::Executor& executor;
        std::promise<void>& release;
        std::thread worker;
        Spin(rclcpp::Executor& e, std::promise<void>& r)
            : executor(e), release(r), worker([&e] { e.spin(); }) {}
        ~Spin() { release.set_value(); executor.cancel(); worker.join(); }
    } spin(executor, release);
    auto client = client_node->create_client<hk_camera::srv::GetPTZPose>("/hk_camera/get_ptz_pose");
    ASSERT_TRUE(client->wait_for_service(3s));
    std::promise<void> entered;
    auto entering = entered.get_future();
    rclcpp::TimerBase::SharedPtr blocker;
    blocker = camera->create_wall_timer(1ms, [&] {
        blocker->cancel();
        entered.set_value();
        // Simulate slow image preparation/response delivery, after the SDK
        // capture call released its lock, using the actual service's group.
        released.wait();
    }, capture_group);
    ASSERT_EQ(entering.wait_for(2s), std::future_status::ready);
    auto response = client->async_send_request(std::make_shared<hk_camera::srv::GetPTZPose::Request>());
    ASSERT_EQ(response.wait_for(400ms), std::future_status::ready);
    EXPECT_EQ(response.get()->message, "Not logged in");
}
