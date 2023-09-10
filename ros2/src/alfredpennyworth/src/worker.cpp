#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("backend", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "brokered",10,[this](const std_msgs::msg::String::SharedPtr msg) {
                //RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
                // Do some stuff
                // Send a reply back to the client
                std_msgs::msg::String reply_msg;
                reply_msg.data = "reply";
                publisher_->publish(reply_msg);
                RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
            }
        );
    }

private:
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
