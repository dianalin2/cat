#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial_handler.hpp"

using namespace std::chrono_literals;

class SerialPublisher : public rclcpp::Node
{
public:
    SerialPublisher()
        : Node("serial_publisher"), serialHandler("/dev/ttyUSB0", 115200)
    {
        // subscriber = this->create_subscription<std_msgs::msg::String>("serial_out", 10);
        publisher = this->create_publisher<std_msgs::msg::String>("serial_in", 10);
        timer = this->create_wall_timer(
            100ms, std::bind(&SerialPublisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    SerialHandler serialHandler;

    void timer_callback()
    {
        try
        {
            std_msgs::msg::String::SharedPtr msg = std::make_shared<std_msgs::msg::String>();
            msg->data = serialHandler.readData();
            RCLCPP_INFO(this->get_logger(), "Read from serial: '%s'", msg->data.c_str());
            publisher->publish(*msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialPublisher>());
    rclcpp::shutdown();
    return 0;
}
