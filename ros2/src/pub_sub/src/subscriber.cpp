// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node{
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

public:
  MinimalSubscriber()
  : Node("minimal_subscriber"){
    count_ = 0;

    //qos.keep_last(10);
    //qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    //qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "frontend", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("backend", 10);
  }

private:
  //funzione richiamata da subscriber
  void topic_callback(const std_msgs::msg::String::SharedPtr msg){

    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    auto message = std_msgs::msg::String();
    message.data = "Pong of " + msg->data;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}