#include <QApplication>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.hpp"
#include "uav_control_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    UAVFleetGUI gui;

    // Start ROS2 node in a separate thread
    QThread* ros_thread = QThread::create([]() {
        auto node = std::make_shared<UAVControlNode>();
        rclcpp::spin(node);
    });
    ros_thread->start();

    gui.show();
    int result = app.exec();

    rclcpp::shutdown();
    ros_thread->quit();
    ros_thread->wait();

    return result;
}