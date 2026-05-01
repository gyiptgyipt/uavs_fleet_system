#include <QApplication>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow.hpp"
#include "uav_control_node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UAVControlNode>();

    QApplication app(argc, argv);
    UAVFleetGUI gui;
    gui.setControlNode(node.get());

    QThread* ros_thread = QThread::create([node]() {
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
