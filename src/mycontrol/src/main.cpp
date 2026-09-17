#include "main.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node_options.hpp>
#include <unistd.h>

namespace control
{
    Control::Control(const std::string &name)
        : Node(name)
    {
        InitParameter();
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            { this->send_data_callback(msg); });
        // 串口初始化
        LibXR::PlatformInit();
        peripherals_ = std::make_unique<LibXR::HardwareContainer>();
        ramfs_ = std::make_unique<LibXR::RamFS>();

        uart_client_ = std::make_unique<LibXR::LinuxUART>(
            vid_, pid_, 115200, LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
        terminal_ = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs_);
        term_thread_ = std::make_unique<LibXR::Thread>();
        term_thread_->Create(terminal_.get(),
                             LibXR::Terminal<1024, 64, 16, 128>::ThreadFun,
                             "terminal", 81900, LibXR::Thread::Priority::MEDIUM);

        static LibXR::HardwareContainer peripherals{
            LibXR::Entry<LibXR::RamFS>({*ramfs_, {"ramfs"}}),
            LibXR::Entry<LibXR::UART>({*uart_client_, {"uart_client"}}),
        };

        // 创建Topic
        // LibXR::Topic::Domain domain("libxr_def_domain");
        wheel = LibXR::Topic::CreateTopic<WheelMsg>("chassis_data");

        XRobotMain(peripherals);
        // 注册接收回调
        // cb0 = LibXR::Topic::Callback::Create(
        //     [](bool, CarPublisher *self, const WheelMsg &msg) {
        //       std::cout << msg.speed_x << " " << msg.speed_y << " " << msg.ang_z
        //                 << std::endl;
        //     }, this);
        // wheel.RegisterCallback(cb0);
    }

    void Control::send_data_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg_data)
    {
        data_.speed_x = msg_data->linear.x;
        data_.speed_y = msg_data->linear.y;
        data_.ang_z = msg_data->angular.z;
        RCLCPP_INFO(this->get_logger(), "x:%f,y:%f,z:%f", data_.speed_x, data_.speed_y, data_.ang_z);
        wheel.Publish(data_);
    }

    void Control::InitParameter()
    {
        vid_ = this->declare_parameter<std::string>("vid", "1a86");
        pid_ = this->declare_parameter<std::string>("pid", "7523");

        RCLCPP_INFO(this->get_logger(), "vid:%s,pid:%s", vid_.c_str(), pid_.c_str());
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pub_node = std::make_shared<control::Control>("control_node");
    rclcpp::spin(pub_node);
    rclcpp::shutdown();
    return 0;
}