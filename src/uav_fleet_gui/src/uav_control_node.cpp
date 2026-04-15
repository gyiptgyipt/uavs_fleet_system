#include "uav_control_node.hpp"

UAVControlNode::UAVControlNode()
    : Node("uav_control_node") {
    vehicle_command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/uav_command", 10);
    setpoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/uav_setpoint", 10);

    RCLCPP_INFO(this->get_logger(), "UAV Control Node started");
}

void UAVControlNode::armVehicle(int vehicle_id) {
    std_msgs::msg::String msg;
    msg.data = QString("ARM %1").arg(vehicle_id).toStdString();
    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Arming vehicle %d", vehicle_id);
}

void UAVControlNode::sendSetpoint(int vehicle_id, double x, double y, double z, double yaw) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    // For simplicity, not setting orientation
    setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sending setpoint to vehicle %d: %f, %f, %f", vehicle_id, x, y, z);
}