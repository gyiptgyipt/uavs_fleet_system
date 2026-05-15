#include "uav_control_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

UAVControlNode::UAVControlNode()
    : Node("uav_control_node") {
    configureFleetParticipants(1);

    setpoint_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            for (const auto& item : setpoint_stream_enabled_) {
                if (item.second) {
                    publishOffboardAndSetpoint(item.first);
                }
            }
        });

    RCLCPP_INFO(this->get_logger(), "UAV Control Node started with PX4 message publishers");
}

void UAVControlNode::configureFleetParticipants(int uav_count) {
    const int bounded_count = std::max(1, uav_count);
    for (int uav_id = 1; uav_id <= bounded_count; ++uav_id) {
        ensurePublishers(uav_id);
        ensureHeartbeatSubscription(uav_id);
        ensurePositionSubscriptions(uav_id);
        if (setpoint_stream_enabled_.count(uav_id) == 0) {
            setpoint_stream_enabled_[uav_id] = false;
        }
    }
}

void UAVControlNode::armVehicle(int vehicle_id) {
    ensurePublishers(vehicle_id);
    ensureHeartbeatSubscription(vehicle_id);
    publishVehicleCommand(
        vehicle_id,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0f);

    RCLCPP_INFO(this->get_logger(), "Arm command sent to UAV %d", vehicle_id);
}

void UAVControlNode::sendSetpoint(int vehicle_id, double x, double y, double z, double yaw) {
    ensurePublishers(vehicle_id);
    ensureHeartbeatSubscription(vehicle_id);

    setpoint_cache_[vehicle_id] = {
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(z),
        static_cast<float>(yaw)
    };
    setpoint_stream_enabled_[vehicle_id] = true;
    publishOffboardAndSetpoint(vehicle_id);
    publishVehicleCommand(
        vehicle_id,
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0f,
        6.0f);

    RCLCPP_INFO(
        this->get_logger(),
        "Setpoint stream enabled for UAV %d: x=%f y=%f z=%f yaw=%f",
        vehicle_id,
        x,
        y,
        z,
        yaw);
}

bool UAVControlNode::sendGeoSetpoint(
    int vehicle_id,
    double latitude,
    double longitude,
    double altitude_m,
    std::string* error_message) {
    ensurePublishers(vehicle_id);
    ensureHeartbeatSubscription(vehicle_id);
    ensurePositionSubscriptions(vehicle_id);

    px4_msgs::msg::VehicleGlobalPosition global_position;
    px4_msgs::msg::VehicleLocalPosition local_position;
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        const auto global_it = last_global_position_.find(vehicle_id);
        const auto local_it = last_local_position_.find(vehicle_id);
        if (global_it == last_global_position_.end() || local_it == last_local_position_.end()) {
            if (error_message) {
                *error_message = "waiting for GPS and local position from PX4";
            }
            return false;
        }
        global_position = global_it->second;
        local_position = local_it->second;
    }

    double current_latitude = static_cast<double>(global_position.lat);
    double current_longitude = static_cast<double>(global_position.lon);
    if (std::abs(current_latitude) > 180.0 || std::abs(current_longitude) > 180.0) {
        current_latitude /= 1e7;
        current_longitude /= 1e7;
    }

    const double earth_radius_m = 6378137.0;
    const double dlat = (latitude - current_latitude) * M_PI / 180.0;
    const double dlon = (longitude - current_longitude) * M_PI / 180.0;
    const double mean_latitude = ((latitude + current_latitude) / 2.0) * M_PI / 180.0;
    const double east = earth_radius_m * dlon * std::cos(mean_latitude);
    const double north = earth_radius_m * dlat;

    const double target_x = static_cast<double>(local_position.x) + north;
    const double target_y = static_cast<double>(local_position.y) + east;
    const double target_z = -std::abs(altitude_m);
    sendSetpoint(vehicle_id, target_x, target_y, target_z, 0.0);
    return true;
}

void UAVControlNode::ensurePublishers(int vehicle_id) {
    if (vehicle_command_publishers_.count(vehicle_id) == 0) {
        const std::string prefix = topicPrefixIn(vehicle_id);
        vehicle_command_publishers_[vehicle_id] =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(prefix + "/vehicle_command", 10);
        trajectory_setpoint_publishers_[vehicle_id] =
            this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(prefix + "/trajectory_setpoint", 10);
        offboard_control_mode_publishers_[vehicle_id] =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(prefix + "/offboard_control_mode", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "Created PX4 publishers for UAV %d on namespace %s",
            vehicle_id,
            prefix.c_str());
    }
}

void UAVControlNode::trackHeartbeat(int vehicle_id) {
    ensureHeartbeatSubscription(vehicle_id);
}

std::optional<int64_t> UAVControlNode::heartbeatAgeMs(int vehicle_id) {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    const auto it = last_heartbeat_time_.find(vehicle_id);
    if (it == last_heartbeat_time_.end()) {
        return std::nullopt;
    }
    const auto age = this->get_clock()->now() - it->second;
    return static_cast<int64_t>(age.nanoseconds() / 1000000);
}

void UAVControlNode::ensureHeartbeatSubscription(int vehicle_id) {
    if (vehicle_status_subscriptions_.count(vehicle_id) != 0) {
        return;
    }

    const std::string topic = topicPrefixOut(vehicle_id) + "/vehicle_status";
    vehicle_status_subscriptions_[vehicle_id] = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        topic,
        10,
        [this, vehicle_id](const px4_msgs::msg::VehicleStatus::SharedPtr /*msg*/) {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            last_heartbeat_time_[vehicle_id] = this->get_clock()->now();
        });

    RCLCPP_INFO(
        this->get_logger(),
        "Tracking heartbeat for UAV %d on topic %s",
        vehicle_id,
        topic.c_str());
}

void UAVControlNode::ensurePositionSubscriptions(int vehicle_id) {
    if (vehicle_global_position_subscriptions_.count(vehicle_id) == 0) {
        const std::string topic = topicPrefixOut(vehicle_id) + "/vehicle_global_position";
        vehicle_global_position_subscriptions_[vehicle_id] =
            this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
                topic,
                10,
                [this, vehicle_id](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    last_global_position_[vehicle_id] = *msg;
                });

        RCLCPP_INFO(
            this->get_logger(),
            "Tracking global position for UAV %d on topic %s",
            vehicle_id,
            topic.c_str());
    }

    if (vehicle_local_position_subscriptions_.count(vehicle_id) == 0) {
        const std::string topic = topicPrefixOut(vehicle_id) + "/vehicle_local_position";
        vehicle_local_position_subscriptions_[vehicle_id] =
            this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
                topic,
                10,
                [this, vehicle_id](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    last_local_position_[vehicle_id] = *msg;
                });

        RCLCPP_INFO(
            this->get_logger(),
            "Tracking local position for UAV %d on topic %s",
            vehicle_id,
            topic.c_str());
    }
}

void UAVControlNode::publishVehicleCommand(int vehicle_id, uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(vehicle_id);
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = nowMicros();
    vehicle_command_publishers_[vehicle_id]->publish(msg);
}

void UAVControlNode::publishOffboardAndSetpoint(int vehicle_id) {
    const auto it = setpoint_cache_.find(vehicle_id);
    if (it == setpoint_cache_.end()) {
        return;
    }

    px4_msgs::msg::OffboardControlMode offboard_msg;
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_msg.timestamp = nowMicros();
    offboard_control_mode_publishers_[vehicle_id]->publish(offboard_msg);

    px4_msgs::msg::TrajectorySetpoint setpoint_msg;
    setpoint_msg.position = {it->second[0], it->second[1], it->second[2]};
    setpoint_msg.yaw = it->second[3];
    setpoint_msg.timestamp = nowMicros();
    trajectory_setpoint_publishers_[vehicle_id]->publish(setpoint_msg);
}

std::string UAVControlNode::topicPrefixIn(int vehicle_id) const {
    return "/px4_" + std::to_string(vehicle_id) + "/fmu/in";
}

std::string UAVControlNode::topicPrefixOut(int vehicle_id) const {
    return "/px4_" + std::to_string(vehicle_id) + "/fmu/out";
}

uint64_t UAVControlNode::nowMicros() {
    return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000);
}
