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
#include "plotter_time/msg/data.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        publisher_ = this->create_publisher<plotter_time::msg::Data>("backend", 10);
        subscriber_ = this->create_subscription<plotter_time::msg::Data>("brokered", 10,
            std::bind(&MyNode::handle_message, this, std::placeholders::_1));
    }

private:
    void handle_message(const plotter_time::msg::Data::SharedPtr msg){
        plotter_time::msg::Data reply_msg;
        reply_msg.data.push_back(123);
        publisher_->publish(reply_msg);
        RCLCPP_INFO(this->get_logger(), "Received message: %d", msg->data[0]);
    }

    rclcpp::Publisher<plotter_time::msg::Data>::SharedPtr publisher_;
    rclcpp::Subscription<plotter_time::msg::Data>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
