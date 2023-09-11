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
        // Processa i messaggi ricevuti dal cliente e li inoltra al worker

        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Msg brokered");
    }

    void handle_backend_message(const std_msgs::msg::String::SharedPtr msg) {
        // Processa i messaggi ricevuti dal worker e li inoltra al cliente

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
