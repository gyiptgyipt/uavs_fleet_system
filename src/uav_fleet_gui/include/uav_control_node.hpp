#ifndef UAV_CONTROL_NODE_HPP
#define UAV_CONTROL_NODE_HPP

#include <cstdint>
#include <map>
#include <string>
#include <array>
#include <mutex>
#include <optional>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

class UAVControlNode : public rclcpp::Node {
public:
    UAVControlNode();
    void configureFleetParticipants(int uav_count);
    void armVehicle(int vehicle_id);
    void sendSetpoint(int vehicle_id, double x, double y, double z, double yaw);
    bool sendGeoSetpoint(int vehicle_id, double latitude, double longitude, double altitude_m, std::string* error_message = nullptr);
    void trackHeartbeat(int vehicle_id);
    std::optional<int64_t> heartbeatAgeMs(int vehicle_id);

private:
    using VehicleCommandPublisher = rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr;
    using TrajectorySetpointPublisher = rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr;
    using OffboardControlModePublisher = rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr;
    using VehicleStatusSubscription = rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr;
    using VehicleGlobalPositionSubscription = rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr;
    using VehicleLocalPositionSubscription = rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr;

    void ensurePublishers(int vehicle_id);
    void ensureHeartbeatSubscription(int vehicle_id);
    void ensurePositionSubscriptions(int vehicle_id);
    void publishVehicleCommand(int vehicle_id, uint16_t command, float param1, float param2 = 0.0f);
    void publishOffboardAndSetpoint(int vehicle_id);
    std::string topicPrefixIn(int vehicle_id) const;
    std::string topicPrefixOut(int vehicle_id) const;
    uint64_t nowMicros();

    rclcpp::TimerBase::SharedPtr setpoint_timer_;
    std::map<int, VehicleCommandPublisher> vehicle_command_publishers_;
    std::map<int, TrajectorySetpointPublisher> trajectory_setpoint_publishers_;
    std::map<int, OffboardControlModePublisher> offboard_control_mode_publishers_;
    std::map<int, VehicleStatusSubscription> vehicle_status_subscriptions_;
    std::map<int, VehicleGlobalPositionSubscription> vehicle_global_position_subscriptions_;
    std::map<int, VehicleLocalPositionSubscription> vehicle_local_position_subscriptions_;
    std::map<int, std::array<float, 4>> setpoint_cache_;
    std::map<int, bool> setpoint_stream_enabled_;

    mutable std::mutex heartbeat_mutex_;
    std::map<int, rclcpp::Time> last_heartbeat_time_;

    mutable std::mutex position_mutex_;
    std::map<int, px4_msgs::msg::VehicleGlobalPosition> last_global_position_;
    std::map<int, px4_msgs::msg::VehicleLocalPosition> last_local_position_;
};

#endif // UAV_CONTROL_NODE_HPP
