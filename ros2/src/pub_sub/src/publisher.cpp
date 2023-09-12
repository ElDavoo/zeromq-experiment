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

#include <chrono>
#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;

#include "plotter_time/msg/plottime.hpp"
#include "plotter_time/msg/data.hpp"
struct timespec timespec_start, timespec_end; //strutture timespec per campionare l'istante di invio e ricezione

using namespace std::chrono_literals;

static const rmw_qos_profile_t unreliable_qos_profile =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    0, //depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // useless
};

static const rmw_qos_profile_t reliable_qos_profile = 
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10, //depth
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // useless
};

class MinimalPublisher : public rclcpp::Node{
    rclcpp::TimerBase::SharedPtr timer_; //timer per il create_wall_timer
    rclcpp::Publisher<plotter_time::msg::Data>::SharedPtr publisher_; //publisher per Ping-pong
    rclcpp::Publisher<plotter_time::msg::Plottime>::SharedPtr publisher_time; //publisher per plot
    size_t count_; //conta la dimensione del payload da inviare 
    rclcpp::Subscription<plotter_time::msg::Data>::SharedPtr subscription_;

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(unreliable_qos_profile));


public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0){
    
    publisher_ = this->create_publisher<plotter_time::msg::Data>("frontend", 10);
    publisher_time = this->create_publisher<plotter_time::msg::Plottime>("time", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::timer_callback, this));

    subscription_ = this->create_subscription<plotter_time::msg::Data>("backend", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
  }

private:
  //funzione che calcola la differenza tra due istanti di tempo
  void sub_timespec(const struct timespec t1, const struct timespec t2, struct timespec *td)
  {
    td->tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td->tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td->tv_sec > 0 && td->tv_nsec < 0)
    {
        td->tv_nsec += 1000000000; //NS_PER_SECOND
        td->tv_sec--;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0)
    {
        td->tv_nsec -= 1000000000; //NS_PER_SECOND
        td->tv_sec++;
    }
  }

  void timer_callback(){ //funzione richiamata da publisher
    auto message = plotter_time::msg::Data();
    count_ = count_ + 1000;
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%lld'", count_);
    // std::stringstream ss;
    for (size_t i = 0; i < count_; i++) {
      uint8_t random_char =rand() % (0x7E - 0x20) + 0x20; //aggiunge un carattere random al payload
      message.data.push_back(random_char);
    }
    // message.data = ss.str();

    clock_gettime(CLOCK_MONOTONIC, &timespec_start);
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Count: %d", count_);
    RCLCPP_INFO(this->get_logger(), "At launch, time is: %lld", timespec_start.tv_nsec);
  }

  //funzione richiamata da subscriber
  void topic_callback(const plotter_time::msg::Data::SharedPtr) {
    
    clock_gettime(CLOCK_MONOTONIC, &timespec_end);

    //invio il messaggio per plottare i dati
    //ho creato un format di messaggio custom
    //ogni volta invia una coppia di dati: dimensione del payload e round-trip time del pacchetto
    auto message_time = plotter_time::msg::Plottime();
    message_time.dim = count_;
    struct timespec timespec_diff;
    sub_timespec(timespec_start, timespec_end, &timespec_diff);
    message_time.time = (timespec_diff.tv_sec * 1000000000.0f) + timespec_diff.tv_nsec;
    publisher_time->publish(message_time);

    RCLCPP_INFO(this->get_logger(), "On reception, time is: %lld", timespec_end.tv_nsec);
  }

};

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
} 