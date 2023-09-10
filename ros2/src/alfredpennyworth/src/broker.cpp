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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RequestReplyBroker : public rclcpp::Node {
public:
    RequestReplyBroker() : Node("request_reply_broker") {
        frontend_ = this->create_subscription<std_msgs::msg::String>("frontend", 10,
            std::bind(&RequestReplyBroker::handle_frontend_message, this, std::placeholders::_1));
        backend_ = this->create_subscription<std_msgs::msg::String>("backend", 10,
            std::bind(&RequestReplyBroker::handle_backend_message, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("brokered", 10);
        publisher_debrok = this->create_publisher<std_msgs::msg::String>("debrokered", 10);
    }

private:
    void handle_frontend_message(const std_msgs::msg::String::SharedPtr msg) {
        // Process the message from the frontend and forward it to the backend
        //std_msgs::msg::String response_msg;
        //response_msg.data = msg->data;
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Msg brokered");
    }

    void handle_backend_message(const std_msgs::msg::String::SharedPtr msg) {
        // Process the message from the backend and forward it to the frontend
        //std_msgs::msg::String response_msg;
        //response_msg.data = msg->data;
        publisher_debrok->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Msg debrokered");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr frontend_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr backend_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_debrok;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RequestReplyBroker>());
    rclcpp::shutdown();
    return 0;
}
