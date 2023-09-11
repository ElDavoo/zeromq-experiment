/*
 * Copyright (C) 2023 by Antonio Solida e Davide Palma
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("backend", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>("brokered", 10,
            std::bind(&MyNode::handle_message, this, std::placeholders::_1));
    }

private:
    void handle_message(const std_msgs::msg::String::SharedPtr msg){
        std_msgs::msg::String reply_msg;
        reply_msg.data = "reply";
        publisher_->publish(reply_msg);
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
