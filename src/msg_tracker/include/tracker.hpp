// classes for tracker objects added to tracked autoware nodes
// design of tracking system: https://docs.google.com/drawings/d/1Gz49K65o4oCqNO7q4g0ol7-B9UKv9zTaE9zuxvcE7sU/edit?usp=sharing

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <deque>
#include <rmw/types.h>
#include <lttng/tracepoint.h>
#include "tracker_tp.h"
#include <string>
#include <iostream>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/empty.hpp>
#include <pthread.h>

#define NS_PER_SEC 1000000000

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

inline uint64_t time2int(builtin_interfaces::msg::Time t) {
  return ((uint64_t)t.sec * NS_PER_SEC) + (uint64_t)t.nanosec;
}

inline builtin_interfaces::msg::Time int2time(uint64_t t) {
  builtin_interfaces::msg::Time time;
  time.sec = t / NS_PER_SEC;
  time.nanosec = t - time.sec * NS_PER_SEC;
  return time;
}

template<typename MsgT>
class PubTracker {
  friend class NodeTracker;
  PubTracker(rmw_gid_t pub_id) : pub_id(pub_id) {}

public:

  const rmw_gid_t pub_id;

  inline void track_publish(const MsgT &msg) const {
    tracepoint(tracker, publish, &pub_id, time2int(msg.header.stamp), gettid());
  }
};

template<typename MsgT>
class SubTracker {
  friend class NodeTracker;
  SubTracker(const char *node_name, const char *topic_name) : sub_id(hash_combine(id_hash(node_name), id_hash(topic_name))) {}

public:

  const uint32_t sub_id;

  inline void track_recieve(const MsgT &msg, const rclcpp::MessageInfo &info) const {
    tracepoint(tracker, recieve, &info.get_rmw_message_info().publisher_gid, sub_id, time2int(msg.header.stamp), gettid());
  }
};

// tracks a ros2 node and any publisher/subscriber trackers related to it
// allows parser to trigger pub_init and sub_init tracepoints
class NodeTracker {
  rclcpp::Node::SharedPtr node;
  std::deque<std::pair<rmw_gid_t, const char *>> tracked_pubs; // pub_id, topic name
  std::deque<std::pair<uint32_t, const char *>> tracked_subs; // sub_id, topic name
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> mapping_init_sub;

  inline const char *get_name() const { return node->get_name(); }

public:

  NodeTracker(rclcpp::Node::SharedPtr node) : node(node) {
    // mapping init callback
    mapping_init_sub = node->create_subscription<std_msgs::msg::Empty>(
      "/msg_tracker/mapping_init",
      10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        for (auto &[pub_id, topic_name] : tracked_pubs) {
          tracepoint(tracker, pub_init, &pub_id, get_name(), topic_name);
        }
        for (auto &[sub_id, topic_name] : tracked_subs) {
          tracepoint(tracker, sub_init, sub_id, get_name(), topic_name);
        }
      }
    );
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
