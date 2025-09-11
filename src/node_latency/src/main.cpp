#include "tracker.hpp"

#include <node_latency_interfaces/msg/detail/generic__struct.hpp>
#include <rclcpp/executors/no_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include "node_latency_interfaces/msg/generic.hpp"
#include <unistd.h>

// simulate components of autoware hotpath
/*
node lidar_top timer -> topic filter_top ---\ 
node lidar_left timer -> topic filter_left -----> node concatenate_data
node lidar_right timer -> topic filter_right -/           /
       /-------------------------------------------------/
       v
topic sensing_pointcloud
node obstacle_segmentation
topic perception_pointcloud
node node object_recognition
topic objects ------------------------------------------------------------------------------------ node behavior_velocity_planner -> (indirect link objects -> path)
  v                                                                                                      ^                                 |
node behavior_path_planner -> (indirect link objects -> path_with_lane_id) -> topic path_with_lane_id ---/                                 |
     /-------------------------------------------------------------------------------------------------------------------------------------/
     v
topic path
node motion_planning
topic trajectory
node motion_velocity_planner

test both inter and intra process
*/

using namespace node_latency_interfaces;

using callback_t = std::pair<std::string, std::function<void(msg::Generic::SharedPtr, const rclcpp::MessageInfo &)>>;

void add_callbacks(std::shared_ptr<rclcpp::Node> &node, callback_t cb) {
    node->create_subscription<msg::Generic>(cb.first, 10, [cb](msg::Generic::SharedPtr msg, const rclcpp::MessageInfo &info) {
        cb.second(msg, info);
    });
}

template<typename... Rest>
void add_callbacks(std::shared_ptr<rclcpp::Node> &node, callback_t cb, Rest... rest) {
    add_callbacks(node, cb);
    add_callbacks(node, rest...);
}

template<typename... Callbacks>
static void launch_node(std::string name, Callbacks... callbacks) {
    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "Failed to spawn process for node " << name << std::endl;
        exit(-1);
    }
    if (pid == 0) return;

    // inside child process
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(name);
    rclcpp::executors::NoExecutor exec;
    add_callbacks(node, callbacks...);
    exec.add_node(node);
    exec.spin();
    exit(0);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pub_sub_test");
    auto node_tracker = std::make_shared<NodeTracker>(node);

    auto pub = node->create_publisher<msg::Generic>("chatter", 10);
    auto chatter_pt = node_tracker->add_pub_tracker<msg::Generic>("chatter", pub->get_gid());
    auto chatter_st = node_tracker->add_sub_tracker<msg::Generic>("chatter");

    auto sub = node->create_subscription<msg::Generic>(
        "chatter",
        10,
        [chatter_st](msg::Generic::SharedPtr msg, const rclcpp::MessageInfo &info) {
            chatter_st->track_recieve(*msg.get(), info);
            RCLCPP_INFO(rclcpp::get_logger("Subscriber"),
                        "I heard: '%s'", msg->data.c_str());
        });

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(500),
        [pub, chatter_pt]() {
            auto msg = msg::Generic();
            msg.header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            msg.data = "Hello ROS 2!";
            RCLCPP_INFO(rclcpp::get_logger("Publisher"),
                        "Publishing: '%s'", msg.data.c_str());
            chatter_pt->track_publish(msg);
            pub->publish(msg);
        });

    rclcpp::executors::NoExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
