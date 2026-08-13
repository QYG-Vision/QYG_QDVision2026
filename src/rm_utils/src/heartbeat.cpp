// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_utils/heartbeat.hpp"

namespace qd {
HeartBeatPublisher::SharedPtr HeartBeatPublisher::create(rclcpp::Node *node) {
  return std::shared_ptr<HeartBeatPublisher>(new HeartBeatPublisher(node));
}

HeartBeatPublisher::HeartBeatPublisher(rclcpp::Node *node)
    : node_(node), last_time_(node->now()) {
  // Initialize message
  message_.data = 0;
  // Create publisher
  std::string node_name = node->get_name();
  std::string topic_name = node_name + "/heartbeat";
  publisher_ = node->create_publisher<std_msgs::msg::Int64>(topic_name, 1);
}

void HeartBeatPublisher::publish() {
  count_++;
  auto now = node_->now();
  if ((now - last_time_).seconds() >= 1.0) {
    message_.data = count_;
    publisher_->publish(message_);
    count_ = 0;
    last_time_ = now;
  }
}

}  // namespace qd
