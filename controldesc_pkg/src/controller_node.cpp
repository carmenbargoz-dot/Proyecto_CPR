#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class DecentralizedController : public rclcpp::Node
{
public:
    DecentralizedController() : Node("decentralized_controller")
    {
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DecentralizedController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de control descentralizado iniciado");
    }

private:
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        // EJEMPLO simple: avanzar recto + girar un poco
        cmd.linear.x  = 0.20;
        cmd.angular.z = 0.15;

        pub_cmd_vel_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecentralizedController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

