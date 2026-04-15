#ifndef UAV_CONTROL_NODE_HPP
#define UAV_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class UAVControlNode : public rclcpp::Node {
public:
    UAVControlNode();
    void armVehicle(int vehicle_id);
    void sendSetpoint(int vehicle_id, double x, double y, double z, double yaw);

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
};

#endif // UAV_CONTROL_NODE_HPP