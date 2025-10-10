// classes for tracker objects added to tracked autoware nodes
// design of tracking system: https://docs.google.com/drawings/d/1Gz49K65o4oCqNO7q4g0ol7-B9UKv9zTaE9zuxvcE7sU/edit?usp=sharing
// warning: without lttng-ust-fork (doesnt seem to exist on Ubuntu), use of fork() breaks tracepoint emissions

#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <deque>
#include <lttng/tracepoint.h>
#include "tracker_tp.h"
#include <string>
#include <iostream>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rmw/types.h>
#include <pthread.h>

#define NS_PER_SEC 1000000000

// hashes node/topic name to an int
inline uint64_t id_hash(const char* str) {
    std::hash<std::string> hash;
    return hash(std::string(str));
}

inline uint64_t hash_combine(uint64_t a, uint64_t b) {
  return a ^ (b + 0x9e3779b97f4a7c15ULL + (a << 6) + (a >> 2));
}

inline uint64_t time2int(builtin_interfaces::msg::Time t) {
  return ((uint64_t)t.sec * NS_PER_SEC) + (uint64_t)t.nanosec;
}

inline uint64_t time2int(rmw_time_t t) {
  return ((uint64_t)t.sec * NS_PER_SEC) + (uint64_t)t.nsec;
}

inline builtin_interfaces::msg::Time int2time(uint64_t t) {
  builtin_interfaces::msg::Time time;
  time.sec = t / NS_PER_SEC;
  time.nanosec = t - time.sec * NS_PER_SEC;
  return time;
}

template<typename IDType>
struct TrackerBase {
  bool enabled = true; // TODO: toggle depending on hotpath and tracking
  IDType id;

  TrackerBase(IDType id) : id(id) {}
};

template<typename MsgT>
class PubTracker : public TrackerBase<rmw_gid_t> {
  friend class NodeTracker;
  PubTracker(rmw_gid_t pub_id) : TrackerBase<rmw_gid_t>(pub_id) {}

public:

  std::atomic<uint64_t> seq_num = 1; // should be synced with publisher's next publication sequence number

  inline void track_publish() {
    if (!enabled) return;

    uint64_t msg_seq_num = seq_num++;
    printf("MESSAGE PUBLISHED seq_num=%llu", msg_seq_num);
    printf("publisher_gid.data: ", id.implementation_identifier);
    for (uint8_t v : id.data) {
      printf("%x ", v);
    }
    printf("\n");
    tracepoint(tracker, publish, &id, msg_seq_num, gettid());
  }
};

template<typename MsgT>
class SubTracker : public TrackerBase<uint64_t> {
  friend class NodeTracker;
  SubTracker(const char *node_name, const char *topic_name) : TrackerBase<uint64_t>(hash_combine(id_hash(node_name), id_hash(topic_name))) {}

public:

  inline void track_receive(const MsgT &msg, const rclcpp::MessageInfo &info) const {
    if (!enabled) return;

    auto rmw_info = info.get_rmw_message_info();
    printf("MESSAGE RECIEVED seq_num=%llu addr=%.016x\n", rmw_info.publication_sequence_number, (uint64_t)(&msg));
    printf("from_intra_process: %d\npublication_sequence_num: %llu\nreception_sequence_number: %llu\npublisher_gid.implementation_identifier: %s\npublisher_gid.data: ", rmw_info.from_intra_process, rmw_info.publication_sequence_number, rmw_info.reception_sequence_number, rmw_info.publisher_gid.implementation_identifier);
    for (uint8_t v : rmw_info.publisher_gid.data) {
      printf("%x ", v);
    }
    printf("\nsource_timestamp: %llu\nreceived_timestamp: %llu\n", rmw_info.source_timestamp, rmw_info.received_timestamp);
    tracepoint(tracker, receive, &info.get_rmw_message_info().publisher_gid, id, rmw_info.publication_sequence_number, gettid());
  }
};

// tracks a ros2 node and any publisher/subscriber trackers related to it
// allows parser to trigger pub_init and sub_init tracepoints
class NodeTracker {
  std::deque<std::pair<std::shared_ptr<TrackerBase<rmw_gid_t>>, std::string>> tracked_pubs; // pub, topic name
  std::deque<std::pair<std::shared_ptr<TrackerBase<uint64_t>>, std::string>> tracked_subs; // sub, topic name
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> mapping_init_sub;
  const char *name;

  inline const char *get_name() const { return name; }

public:

  template<typename NodeT>
  NodeTracker(NodeT &node) : name(node.get_name()) {
    // mapping init callback
    mapping_init_sub = node.template create_subscription<std_msgs::msg::Empty>(
      "/msg_tracker/mapping_init",
      10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        for (auto &[pub_tracker, topic_name] : tracked_pubs) {
          tracepoint(tracker, pub_init, &pub_tracker->id, get_name(), topic_name.c_str());
        }
        for (auto &[sub_tracker, topic_name] : tracked_subs) {
          tracepoint(tracker, sub_init, sub_tracker->id, get_name(), topic_name.c_str());
        }
      }
    );
  }

  template<typename MsgT>
  std::shared_ptr<PubTracker<MsgT>> add_pub_tracker(const char *topic_name, rmw_gid_t pub_id) {
    auto t = std::shared_ptr<PubTracker<MsgT>>(new PubTracker<MsgT>(pub_id));
    tracked_pubs.emplace_back(t, topic_name);
    return t;
  }

  template<typename MsgT>
  std::shared_ptr<SubTracker<MsgT>> add_sub_tracker(const char *topic_name) {
    auto t = std::shared_ptr<SubTracker<MsgT>>(new SubTracker<MsgT>(get_name(), topic_name));
    tracked_subs.emplace_back(t, topic_name);
    return t;
  }
};
