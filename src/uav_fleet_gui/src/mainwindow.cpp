#include "mainwindow.hpp"
#include "uav_control_node.hpp"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QProcess>
#include <QDebug>

SimulationThread::SimulationThread(const QString& command)
    : command_(command) {}

void SimulationThread::run() {
    QProcess process;
    process.start("bash", QStringList() << "-c" << command_);
    process.waitForFinished(-1);
    QString output = process.readAllStandardOutput();
    QString error = process.readAllStandardError();
    emit outputReceived(output + error);
}

UAVFleetGUI::UAVFleetGUI(QWidget *parent)
    : QMainWindow(parent), node_(nullptr), sim_thread_(nullptr) {
    setupUI();

    telemetry_timer_ = new QTimer(this);
    connect(telemetry_timer_, &QTimer::timeout, this, &UAVFleetGUI::updateTelemetry);
    telemetry_timer_->start(1000);
}

UAVFleetGUI::~UAVFleetGUI() {
    if (sim_thread_ && sim_thread_->isRunning()) {
        sim_thread_->terminate();
        sim_thread_->wait();
    }
}

void UAVFleetGUI::setupUI() {
    setWindowTitle("UAV Fleet Control GUI - PX4 & ROS2");
    setGeometry(100, 100, 1200, 800);

    QWidget* central_widget = new QWidget;
    setCentralWidget(central_widget);

    QHBoxLayout* main_layout = new QHBoxLayout(central_widget);

    // Left panel
    QWidget* left_panel = createLeftPanel();
    main_layout->addWidget(left_panel);

    // Right panel
    QTabWidget* right_panel = createRightPanel();
    main_layout->addWidget(right_panel);

    main_layout->setStretchFactor(left_panel, 1);
    main_layout->setStretchFactor(right_panel, 3);
}

QWidget* UAVFleetGUI::createLeftPanel() {
    QWidget* panel = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(panel);

    // UAV List Group
    QGroupBox* uav_group = new QGroupBox("UAV Fleet");
    QVBoxLayout* uav_layout = new QVBoxLayout(uav_group);

    uav_list_ = new QListWidget;
    uav_list_->addItem("UAV 1");
    uav_list_->addItem("UAV 2");
    uav_layout->addWidget(uav_list_);

    // Status Table
    status_table_ = new QTableWidget;
    status_table_->setColumnCount(3);
    status_table_->setHorizontalHeaderLabels({"UAV ID", "Status", "Battery"});
    status_table_->setRowCount(2);
    status_table_->setItem(0, 0, new QTableWidgetItem("1"));
    status_table_->setItem(0, 1, new QTableWidgetItem("Disconnected"));
    status_table_->setItem(0, 2, new QTableWidgetItem("N/A"));
    status_table_->setItem(1, 0, new QTableWidgetItem("2"));
    status_table_->setItem(1, 1, new QTableWidgetItem("Disconnected"));
    status_table_->setItem(1, 2, new QTableWidgetItem("N/A"));
    uav_layout->addWidget(status_table_);

    layout->addWidget(uav_group);

    // Control Buttons
    QGroupBox* control_group = new QGroupBox("Controls");
    QVBoxLayout* control_layout = new QVBoxLayout(control_group);

    start_sim_btn_ = new QPushButton("Start PX4 Simulation");
    connect(start_sim_btn_, &QPushButton::clicked, this, &UAVFleetGUI::startSimulation);
    control_layout->addWidget(start_sim_btn_);

    arm_btn_ = new QPushButton("Arm Selected UAV");
    connect(arm_btn_, &QPushButton::clicked, this, &UAVFleetGUI::armUAVs);
    control_layout->addWidget(arm_btn_);

    setpoint_btn_ = new QPushButton("Send Setpoints");
    connect(setpoint_btn_, &QPushButton::clicked, this, &UAVFleetGUI::sendSetpoints);
    control_layout->addWidget(setpoint_btn_);

    layout->addWidget(control_group);

    return panel;
}

QTabWidget* UAVFleetGUI::createRightPanel() {
    QTabWidget* panel = new QTabWidget;

    panel->addTab(createDashboardTab(), "Dashboard");
    panel->addTab(createMissionTab(), "Mission Planning");
    panel->addTab(createTelemetryTab(), "Telemetry");
    panel->addTab(createLogsTab(), "Logs");

    return panel;
}

QWidget* UAVFleetGUI::createDashboardTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);

    // Map View
    QGroupBox* map_group = new QGroupBox("Live Map View");
    QVBoxLayout* map_layout = new QVBoxLayout(map_group);
    QLabel* map_view = new QLabel("Map integration would go here");
    map_view->setStyleSheet("background-color: lightblue; border: 1px solid black; padding: 20px;");
    map_view->setAlignment(Qt::AlignCenter);
    map_layout->addWidget(map_view);
    layout->addWidget(map_group);

    // Stats
    QGroupBox* stats_group = new QGroupBox("Fleet Statistics");
    QGridLayout* stats_layout = new QGridLayout(stats_group);
    stats_layout->addWidget(new QLabel("Total UAVs:"), 0, 0);
    stats_layout->addWidget(new QLabel("2"), 0, 1);
    stats_layout->addWidget(new QLabel("Active Missions:"), 1, 0);
    stats_layout->addWidget(new QLabel("0"), 1, 1);
    layout->addWidget(stats_group);

    return tab;
}

QWidget* UAVFleetGUI::createMissionTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);

    // Mission Config
    QGroupBox* config_group = new QGroupBox("Mission Configuration");
    QGridLayout* config_layout = new QGridLayout(config_group);

    config_layout->addWidget(new QLabel("Mission Type:"), 0, 0);
    mission_type_ = new QComboBox;
    mission_type_->addItems({"Waypoint Navigation", "Area Survey", "Return to Home", "Custom"});
    config_layout->addWidget(mission_type_, 0, 1);

    config_layout->addWidget(new QLabel("Altitude (m):"), 1, 0);
    altitude_input_ = new QLineEdit("10");
    config_layout->addWidget(altitude_input_, 1, 1);

    layout->addWidget(config_group);

    // Waypoints
    QGroupBox* waypoint_group = new QGroupBox("Waypoints");
    QVBoxLayout* waypoint_layout = new QVBoxLayout(waypoint_group);

    waypoint_table_ = new QTableWidget;
    waypoint_table_->setColumnCount(4);
    waypoint_table_->setHorizontalHeaderLabels({"X", "Y", "Z", "Action"});
    waypoint_layout->addWidget(waypoint_table_);

    QPushButton* add_wp_btn = new QPushButton("Add Waypoint");
    connect(add_wp_btn, &QPushButton::clicked, this, &UAVFleetGUI::addWaypoint);
    waypoint_layout->addWidget(add_wp_btn);

    layout->addWidget(waypoint_group);

    return tab;
}

QWidget* UAVFleetGUI::createTelemetryTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);

    QGroupBox* telemetry_group = new QGroupBox("Real-time Telemetry");
    QGridLayout* telemetry_layout = new QGridLayout(telemetry_group);

    telemetry_layout->addWidget(new QLabel("Selected UAV:"), 0, 0);
    selected_uav_telemetry_ = new QLabel("None");
    telemetry_layout->addWidget(selected_uav_telemetry_, 0, 1);

    telemetry_layout->addWidget(new QLabel("Position:"), 1, 0);
    position_label_ = new QLabel("N/A");
    telemetry_layout->addWidget(position_label_, 1, 1);

    telemetry_layout->addWidget(new QLabel("Velocity:"), 2, 0);
    velocity_label_ = new QLabel("N/A");
    telemetry_layout->addWidget(velocity_label_, 2, 1);

    telemetry_layout->addWidget(new QLabel("Battery:"), 3, 0);
    battery_label_ = new QLabel("N/A");
    telemetry_layout->addWidget(battery_label_, 3, 1);

    layout->addWidget(telemetry_group);

    return tab;
}

QWidget* UAVFleetGUI::createLogsTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);

    output_text_ = new QTextEdit;
    output_text_->setReadOnly(true);
    layout->addWidget(output_text_);

    QHBoxLayout* controls = new QHBoxLayout;

    QPushButton* clear_btn = new QPushButton("Clear Logs");
    connect(clear_btn, &QPushButton::clicked, this, &UAVFleetGUI::clearLogs);
    controls->addWidget(clear_btn);

    QPushButton* save_btn = new QPushButton("Save Logs");
    connect(save_btn, &QPushButton::clicked, this, &UAVFleetGUI::saveLogs);
    controls->addWidget(save_btn);

    layout->addLayout(controls);

    return tab;
}

void UAVFleetGUI::startSimulation() {
    QString command = "cd ~/PX4-Autopilot && ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 2";
    sim_thread_ = new SimulationThread(command);
    connect(sim_thread_, &SimulationThread::outputReceived, this, &UAVFleetGUI::updateOutput);
    sim_thread_->start();
}

void UAVFleetGUI::armUAVs() {
    QListWidgetItem* selected = uav_list_->currentItem();
    if (selected) {
        int uav_id = selected->text().split(" ")[1].toInt();
        // Note: In a real implementation, you'd need to pass the node here
        qDebug() << "Arming UAV" << uav_id;
    }
}

void UAVFleetGUI::sendSetpoints() {
    QListWidgetItem* selected = uav_list_->currentItem();
    if (selected) {
        int uav_id = selected->text().split(" ")[1].toInt();
        qDebug() << "Sending setpoints to UAV" << uav_id;
    }
}

void UAVFleetGUI::addWaypoint() {
    int row = waypoint_table_->rowCount();
    waypoint_table_->insertRow(row);
    waypoint_table_->setItem(row, 0, new QTableWidgetItem("0"));
    waypoint_table_->setItem(row, 1, new QTableWidgetItem("0"));
    waypoint_table_->setItem(row, 2, new QTableWidgetItem("10"));

    QPushButton* delete_btn = new QPushButton("Delete");
    connect(delete_btn, &QPushButton::clicked, [this, row]() { deleteWaypoint(row); });
    waypoint_table_->setCellWidget(row, 3, delete_btn);
}

void UAVFleetGUI::deleteWaypoint(int row) {
    waypoint_table_->removeRow(row);
}

void UAVFleetGUI::startMission() {
    output_text_->append("Starting mission...");
}

void UAVFleetGUI::clearLogs() {
    output_text_->clear();
}

void UAVFleetGUI::saveLogs() {
    output_text_->append("Logs saved...");
}

void UAVFleetGUI::updateTelemetry() {
    QListWidgetItem* selected = uav_list_->currentItem();
    if (selected) {
        selected_uav_telemetry_->setText(selected->text());
        position_label_->setText("0.0, 0.0, 5.0");
        velocity_label_->setText("2.5, 0.0, 0.0");
        battery_label_->setText("85%");
        gps_label_->setText("3D Fix");
    }
}

void UAVFleetGUI::updateOutput(const QString& text) {
    output_text_->append(text);
}