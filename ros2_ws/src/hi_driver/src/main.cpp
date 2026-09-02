#include "rclcpp/rclcpp.hpp"
#include "XingSongDriver/xingsong_driver.h"
using namespace std;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("hi_node");
    
     XingsongDriver xdddriver(node);

     xdddriver.Init();

     xdddriver.Start();

    return 0;
}


