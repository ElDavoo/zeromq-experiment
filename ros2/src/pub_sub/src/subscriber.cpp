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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "plotter_time/msg/data.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node{
  rclcpp::Subscription<plotter_time::msg::Data>::SharedPtr subscription_;
  rclcpp::Publisher<plotter_time::msg::Data>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

public:
  MinimalSubscriber()
  : Node("minimal_subscriber",rclcpp::NodeOptions()){
    count_ = 0;

    subscription_ = this->create_subscription<plotter_time::msg::Data>("frontend", 10, 
      std::bind(&MinimalSubscriber::topic_callback, this, _1));
    publisher_ = this->create_publisher<plotter_time::msg::Data>("backend", 10);
  }

private:
  //funzione richiamata da subscriber
  void topic_callback(const plotter_time::msg::Data::SharedPtr msg){

    auto message = plotter_time::msg::Data();
    message.data.push_back(123);
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data[0]);
  }
  
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}