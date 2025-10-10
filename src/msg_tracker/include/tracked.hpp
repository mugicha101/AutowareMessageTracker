// wrapper classes for rclcpp objects that handle tracking internally

#pragma once

#include "tracker.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

template<typename MessageT, typename AllocatorT = std::allocator<void>>
struct TrackedPublisher : public rclcpp::Publisher<MessageT, AllocatorT> {
  using Base = rclcpp::Publisher<MessageT, AllocatorT>;
  
  std::shared_ptr<PubTracker<MessageT>> tracker; // assigned by TrackedNode

  TrackedPublisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options)
  : rclcpp::Publisher<MessageT, AllocatorT>(node_base, topic, qos, options) {}

  template<typename T>
  auto publish(T&& msg)
  {
    tracker->track_publish();
    return Base::template publish<std::remove_reference_t<T>>(std::forward<T>(msg));
  }

  auto publish(const rcl_serialized_message_t & serialized_msg)
  {
    tracker->track_publish();
    return Base::publish(serialized_msg);
  }

  auto publish(const rclcpp::SerializedMessage & serialized_msg)
  {
    tracker->track_publish();
    return Base::publish(serialized_msg);
  }

  template<typename TMsg = MessageT>
  auto publish(rclcpp::LoanedMessage<TMsg, AllocatorT> && loaned_msg)
  {
    tracker->track_publish();
    return Base::publish(std::move(loaned_msg));
  }
};

// enables tracking on publishers
// subscribers need to be handled separately since hard to do with template metaprogramming
struct TrackedNode : public rclcpp::Node {
  RCLCPP_SMART_PTR_DEFINITIONS(TrackedNode)

  std::shared_ptr<NodeTracker> tracker;

  explicit TrackedNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options), tracker(std::make_shared<NodeTracker>(*this)) {}


  explicit TrackedNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, namespace_, options), tracker(std::make_shared<NodeTracker>(*this)) {}

  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = TrackedPublisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
    rclcpp::PublisherOptionsWithAllocator<AllocatorT>()
  ) {
    auto pub = rclcpp::Node::create_publisher<MessageT, AllocatorT, PublisherT>(topic_name, qos, options);
    pub->tracker = tracker->add_pub_tracker<MessageT>(topic_name.c_str(), pub->get_gid());
    return pub;
  }
};
