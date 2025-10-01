#include "tracker.hpp"

#include <chrono>
#include <node_latency_interfaces/msg/detail/generic__struct.hpp>
// #include <rclcpp/executors/no_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include "node_latency_interfaces/msg/generic.hpp"
#include <cstdio>
#include <unistd.h>
#include <sys/wait.h>

using namespace std::chrono_literals;

// simulate components of autoware hotpath
/*
node lidar_top timer -> topic filter_top ---\ 
node lidar_left timer -> topic filter_left -----> (indirect link) --> node concatenate_data
node lidar_right timer -> topic filter_right -/                         /
       /---------------------------------------------------------------/
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

std::pair<std::shared_ptr<rclcpp::Node>, std::shared_ptr<NodeTracker>> make_node(std::string name) {
    auto node = rclcpp::Node::make_shared(name);
    auto node_tracker = std::make_shared<NodeTracker>(node);
    return {node, node_tracker};
}

std::pair<rclcpp::Publisher<msg::Generic>::SharedPtr, std::shared_ptr<PubTracker<msg::Generic>>> add_pub(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<NodeTracker> node_tracker, std::string topic) {
    auto pub = node->create_publisher<msg::Generic>(topic, 10);
    auto pub_tracker = node_tracker->add_pub_tracker<msg::Generic>(topic.c_str(), pub->get_gid());
    return {pub, pub_tracker};
}

std::pair<rclcpp::Subscription<msg::Generic>::SharedPtr, std::shared_ptr<SubTracker<msg::Generic>>> add_sub(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<NodeTracker> node_tracker, std::string topic, std::function<void(std::shared_ptr<SubTracker<msg::Generic>>, msg::Generic::SharedPtr, const rclcpp::MessageInfo &)> callback) {
    auto sub_tracker = node_tracker->add_sub_tracker<msg::Generic>(topic.c_str());
    auto sub = node->create_subscription<msg::Generic>(
        topic,
        10,
        [callback, sub_tracker](msg::Generic::SharedPtr msg, const rclcpp::MessageInfo &info) {
            callback(sub_tracker, msg, info);
        }
    );
    return {sub, sub_tracker};
}

// LIDAR TOP
void lidar_top() {
    auto [lidar_top, lidar_top_tracker] = make_node("lidar_top");
    auto [filter_top_pub, filter_top_pub_tracker] = add_pub(lidar_top, lidar_top_tracker, "perception/filter_top");
    auto timer = lidar_top->create_wall_timer(
        50ms,
        [&]() {
            static int count;
            auto msg = filter_top_pub->borrow_loaned_message();
            msg.get().header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
            msg.get().data = 0.0;
            filter_top_pub_tracker->track_publish(msg.get());
            std::cout << "PUBLISH " << (uint64_t)&msg.get() << " " << ++count << std::endl;
            filter_top_pub->publish(std::move(msg));
        }
    );

    std::cout << "LIDAR TOP" << std::endl;
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(lidar_top);
    exec.spin();
    rclcpp::shutdown();
}

// CONCATENATE DATA - WAITS FOR 3 POINTCLOUDS OR TIMEOUT BEFORE CONCATTING (no need to model timeout)
void concatenate_data() {
    auto [concatenate_data, concatenate_data_tracker] = make_node("concatenate_data");
    std::shared_ptr<std::vector<uint64_t>> ts = std::make_shared<std::vector<uint64_t>>();
    auto [filter_top_sub, filter_top_sub_tracker] = add_sub(
        concatenate_data, concatenate_data_tracker, "perception/filter_top",
        [&ts](auto sub_tracker, auto msg, auto &info) {
            sub_tracker->track_recieve(*msg.get(), info);
            auto rmw_info = info.get_rmw_message_info();
            std::cout << "RECEIVE " << (uint64_t)(msg.get()) << " " << rmw_info.publication_sequence_number << std::endl;
            ts->push_back(rmw_info.received_timestamp - rmw_info.source_timestamp);
            if (ts->size() != 1000) return;
            
            std::vector<uint64_t> vals = *ts;
            std::sort(vals.begin(), vals.end());
            auto [min_it, max_it] = std::minmax_element(vals.begin(), vals.end());
            uint64_t min_val = *min_it;
            uint64_t max_val = *max_it;
            size_t n = vals.size();
            size_t mid = n >> 1;
            double mean = (double)std::accumulate(vals.begin(), vals.end(), 0) / (double)n;
            double median = n & 1 ? (double)vals[mid] : (double)(vals[mid] + vals[mid + 1]) * 0.5L;
            double stddev = std::sqrt((double)std::accumulate(vals.begin(), vals.end(), 0ULL, [mean](uint64_t acc, uint64_t x) {
                uint64_t dev = (x > mean ? x - mean : mean - x);
                return acc + dev * dev;
            }) / (double)n);
            FILE* fp = std::fopen("latency.txt", "w");
            std::fprintf(fp, "LATENCY STATS\ncount: %llu\nmin: %lluns\nmax: %lluns\nmean: %lluns\nmedian: %lluns\nstddev: %lluns\n", n, min_val, max_val, (uint64_t)round(mean), (uint64_t)round(median), (uint64_t)round(stddev));
            std::fclose(fp);
        }
    );
    auto [sensing_pointcloud_pub, sensing_pointcloud_pub_tracker] = add_pub(concatenate_data, concatenate_data_tracker, "sensing/pointcloud");
    
    std::cout << "CONCAT DATA" << std::endl;
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(concatenate_data);
    exec.spin();
    rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
    if (argc < 2) {
        std::cerr << "node_name parameter not specified" << std::endl;
        return -1;
    }
    rclcpp::init(argc, argv);
    std::string node_name = argv[1];
    if (node_name == "lidar_top") {
        lidar_top();
    } else if (node_name == "concatenate_data") {
        concatenate_data();
    } else {
        std::cerr << "Error - unknown node_name: " << node_name << std::endl;
        return -1;
    }
    return 0;
}
