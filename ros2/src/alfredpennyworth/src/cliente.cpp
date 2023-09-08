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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;

#include "plotter_time/msg/plottime.hpp"
struct timespec timespec_start, timespec_end;

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node{
    rclcpp::TimerBase::SharedPtr timer_; //timer per il create_wall_timer
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; //publisher per Ping-pong
    rclcpp::Publisher<plotter_time::msg::Plottime>::SharedPtr publisher_time; //publisher per plot
    size_t count_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    rcl_time_point_value_t t_pong; //istante di invio del messaggio

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));


public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0){
    //qos settings
    qos.keep_last(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    //end of qos settings
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("frontend", 10);
    publisher_time = this->create_publisher<plotter_time::msg::Plottime>("time", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback, this));

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "debrokered", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
  }

private:
  void timer_callback(){ //funzione richiamata da publisher

    auto message = std_msgs::msg::String();
    count_ = count_ + 1000;
    message.data = /*"Ping " + */std::to_string(count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    for(size_t i = 0;i<count_;i++){
      message.data = message.data + (char) (rand() % (0x7e - 0x20) + 0x20);
    }
    
    publisher_->publish(message);

    //timer stuff
    clock_gettime(CLOCK_MONOTONIC, &timespec_start);
    RCLCPP_INFO(this->get_logger(), "Count: %d", count_);
    RCLCPP_INFO(this->get_logger(), "At launch, time is: %lld", timespec_end.tv_nsec);
  }

  //funzione richiamata da subscriber
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    
    clock_gettime(CLOCK_MONOTONIC, &timespec_end);
    RCLCPP_INFO(this->get_logger(), "On reception, time is: %lld", timespec_end.tv_nsec);
    RCLCPP_INFO(this->get_logger(), "TIME PASSED: %lld", (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000); //converto i ns in sec
    
    //invio il messaggio per plottare i dati
    //ho creato un format di messaggio custom
    //ogni volta invia una coppia di dati: dimensione del payload e round-trip time del pacchetto
    auto message_time = plotter_time::msg::Plottime();
    message_time.time = (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000; //converte il tempo da nanosecondi in secondi
    message_time.dim = count_;
    if (timespec_end.tv_nsec - timespec_start.tv_nsec >= 0) 
      publisher_time->publish(message_time);
    
  }

};

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
