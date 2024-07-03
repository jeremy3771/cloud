#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ddsm_cpp_py/motor_command.hpp" // 올바른 경로로 수정
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control"), motor1(0), motor2(1), motor3(2), motor4(3) {
        subscription = this->create_subscription<std_msgs::msg::String>(
            "/direction",
            10,
            std::bind(&MotorControlNode::direction_callback, this, std::placeholders::_1));
    }

private:
    void direction_callback(const std_msgs::msg::String::SharedPtr msg) {
        auto direction = msg->data.c_str();
        RCLCPP_INFO(this->get_logger(), "Received direction: %s", direction);

        if (std::string(direction) == "Turn Left") {
            // motor1.SET_VELOCITY(1, -50);
            // std::this_thread::sleep_for(5ms);
            // motor1.SET_VELOCITY(2, -50);
            // std::this_thread::sleep_for(5ms);
        } else if (std::string(direction) == "Go Straight") {
            motor1.SET_VELOCITY(1, -50);
            std::this_thread::sleep_for(5ms);
            motor1.SET_VELOCITY(2, 50);
            std::this_thread::sleep_for(5ms);
        } else if (std::string(direction) == "Turn Right") {
            // motor1.SET_VELOCITY(1, 50);
            // std::this_thread::sleep_for(5ms);
            // motor1.SET_VELOCITY(2, 0);
            // std::this_thread::sleep_for(5ms);
        } else if (std::string(direction) == "Go Back") {
            motor1.SET_VELOCITY(1, 50);
            std::this_thread::sleep_for(5ms);
            motor1.SET_VELOCITY(2, -50);
            std::this_thread::sleep_for(5ms);
        } else if (std::string(direction) == "Stop") {
            motor1.SET_VELOCITY(1, 0);
            std::this_thread::sleep_for(5ms);
            motor1.SET_VELOCITY(2, 0);
            std::this_thread::sleep_for(5ms);
        } else {
            RCLCPP_INFO(this->get_logger(), "Unknown direction received.");
        }
    }

    MOTOR_COMMAND motor1, motor2, motor3, motor4;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
