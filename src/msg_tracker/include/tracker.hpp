// classes for tracker objects added to tracked autoware nodes
// design of tracking system: https://docs.google.com/drawings/d/1Gz49K65o4oCqNO7q4g0ol7-B9UKv9zTaE9zuxvcE7sU/edit?usp=sharing

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <deque>
#include <rmw/types.h>
#include "tracepoint_provider.h"
#include <rmw/types.h>
#include <string>
#include <iostream>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/empty.hpp>

// hashes node/topic name to an int
constexpr uint32_t id_hash(const char* str) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; str[i] != '\0'; ++i) {
        hash ^= static_cast<uint32_t>(str[i]);
        hash *= 16777619u;
    }
    return hash;
}

constexpr uint32_t hash_combine(uint32_t seed, uint32_t hash) {
  return seed ^ (hash + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}

std::string gen_uid(const rmw_gid_t &gid, builtin_interfaces::msg::Time src_stamp);

template<typename MsgT>
class PubTracker {
  friend class NodeTracker;
  PubTracker(rmw_gid_t pub_id) : pub_id(pub_id) {}

public:

  const rmw_gid_t pub_id;

  void track_publish(const MsgT &msg) const {
    std::string uid = gen_uid(pub_id, msg.header.stamp);
    std::cout << "PUB: " << uid << std::endl;
  }
};

template<typename MsgT>
class SubTracker {
  friend class NodeTracker;
  SubTracker(const char *node_name, const char *topic_name) : sub_id(hash_combine(id_hash(node_name), id_hash(topic_name))) {}

public:

  const uint32_t sub_id;

  void track_recieve(const MsgT &msg, const rclcpp::MessageInfo &info) const {
    std::string uid = gen_uid(info.get_rmw_message_info().publisher_gid, msg.header.stamp);
    std::cout << "SUB: " << uid << std::endl;
  }
};

// tracks a ros2 node and any publisher/subscriber trackers related to it
// allows UI to trigger pub_init and sub_init tracepoints
class NodeTracker {
  rclcpp::Node::SharedPtr node;
  std::deque<std::pair<rmw_gid_t, const char *>> tracked_pubs; // pub_id, topic name
  std::deque<std::pair<uint32_t, const char *>> tracked_subs; // sub_id, topic name

public:

  NodeTracker(rclcpp::Node::SharedPtr node) : node(node) {
    // ui init callback
    node->create_subscription<std_msgs::msg::Empty>(
      "msg_tracker/ui_init",
      10,
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        for (auto &[pub_id, topic_name] : tracked_pubs) {
          // TODO: emit pub_init tracepoint
        }
        for (auto &[sub_id, topic_name] : tracked_subs) {
          // TODO: emit sub_init tracepoint
        }
      }
    );
  }

  inline const char *get_name() const {
    return node->get_name();
  }

  template<typename MsgT>
  std::shared_ptr<PubTracker<MsgT>> add_pub_tracker(const char *topic_name, rmw_gid_t pub_id) {
    auto t = std::shared_ptr<PubTracker<MsgT>>(new PubTracker<MsgT>(pub_id));
    tracked_pubs.emplace_back(t->pub_id, topic_name);
    return t;
  }

  template<typename MsgT>
  std::shared_ptr<SubTracker<MsgT>> add_sub_tracker(const char *topic_name) {
    auto t = std::shared_ptr<SubTracker<MsgT>>(new SubTracker<MsgT>(get_name(), topic_name));
    tracked_subs.emplace_back(t->sub_id, topic_name);
    return t;
  }
};
