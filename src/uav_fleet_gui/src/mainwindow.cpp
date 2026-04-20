#include "mainwindow.hpp"
#include "uav_control_node.hpp"

#include <QDebug>
#include <QFile>
#include <QFrame>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>

SimulationThread::SimulationThread(const QString& command)
    : command_(command) {}

void SimulationThread::run() {
    QProcess process;
    process.start("bash", QStringList() << "-c" << command_);
    process.waitForFinished(-1);
    const QString output = process.readAllStandardOutput();
    const QString error = process.readAllStandardError();
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
    setWindowTitle("Cloud Fleet Mission Control");
    resize(1440, 920);
    setMinimumSize(1180, 760);

    setStyleSheet(R"(
        QMainWindow {
            background: #f4f7fb;
        }
        QWidget {
            color: #24344d;
            font-family: "Noto Sans", "DejaVu Sans", sans-serif;
            font-size: 14px;
        }
        QFrame#SidebarCard, QFrame#HeroCard, QFrame#MetricCard, QFrame#ContentCard {
            background: #ffffff;
            border: 1px solid #dce5f1;
            border-radius: 22px;
        }
        QLabel#AppEyebrow {
            color: #4c86ff;
            font-size: 12px;
            font-weight: 700;
            letter-spacing: 1px;
            text-transform: uppercase;
        }
        QLabel#SidebarTitle {
            color: #0e1b2a;
            font-size: 28px;
            font-weight: 800;
        }
        QLabel#SidebarSubtitle, QLabel#MutedText {
            color: #65748b;
            font-size: 13px;
        }
        QLabel#StatusPill {
            background: #e9fff1;
            color: #17824f;
            border: 1px solid #bfeccc;
            border-radius: 12px;
            padding: 6px 12px;
            font-size: 12px;
            font-weight: 700;
        }
        QLabel#SectionTitle {
            color: #132238;
            font-size: 18px;
            font-weight: 700;
        }
        QLabel#MetricValue {
            color: #10233f;
            font-size: 28px;
            font-weight: 800;
        }
        QLabel#MetricCaption {
            color: #6c7a90;
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
        }
        QListWidget, QTableWidget, QTextEdit, QLineEdit, QComboBox {
            background: #f8fbff;
            border: 1px solid #d7e2f0;
            border-radius: 16px;
            padding: 8px;
        }
        QListWidget::item {
            padding: 12px;
            margin: 4px 0;
            border-radius: 12px;
        }
        QListWidget::item:selected {
            background: #dce9ff;
            color: #16335f;
        }
        QHeaderView::section {
            background: #eef4fb;
            color: #4f5f76;
            border: none;
            border-bottom: 1px solid #d7e2f0;
            padding: 10px;
            font-weight: 700;
        }
        QPushButton {
            background: #eff4fb;
            color: #17335f;
            border: 1px solid #d5e0ef;
            border-radius: 14px;
            padding: 12px 16px;
            font-weight: 700;
        }
        QPushButton:hover {
            background: #e4eefc;
        }
        QPushButton#PrimaryButton {
            background: #1f6fff;
            color: white;
            border: none;
        }
        QPushButton#PrimaryButton:hover {
            background: #175fdd;
        }
        QTabWidget::pane {
            border: none;
            background: transparent;
        }
        QTabBar::tab {
            background: #eaf0f8;
            color: #526276;
            border: none;
            border-radius: 14px;
            padding: 10px 18px;
            margin-right: 8px;
            font-weight: 700;
        }
        QTabBar::tab:selected {
            background: #17335f;
            color: white;
        }
        QTextEdit {
            padding: 14px;
        }
    )");

    QWidget* central_widget = new QWidget;
    setCentralWidget(central_widget);

    QHBoxLayout* main_layout = new QHBoxLayout(central_widget);
    main_layout->setContentsMargins(24, 24, 24, 24);
    main_layout->setSpacing(24);

    QWidget* left_panel = createLeftPanel();
    left_panel->setFixedWidth(320);
    main_layout->addWidget(left_panel);

    QVBoxLayout* content_layout = new QVBoxLayout;
    content_layout->setSpacing(20);
    main_layout->addLayout(content_layout, 1);

    QFrame* hero_card = new QFrame;
    hero_card->setObjectName("HeroCard");
    QHBoxLayout* hero_layout = new QHBoxLayout(hero_card);
    hero_layout->setContentsMargins(28, 28, 28, 28);
    hero_layout->setSpacing(24);

    QVBoxLayout* hero_text_layout = new QVBoxLayout;
    hero_text_layout->setSpacing(10);

    QLabel* eyebrow = new QLabel("Mission orchestration");
    eyebrow->setObjectName("AppEyebrow");
    hero_text_layout->addWidget(eyebrow);

    QLabel* hero_title = new QLabel("CloudCare-style fleet oversight for PX4 operations");
    hero_title->setWordWrap(true);
    hero_title->setStyleSheet("font-size: 30px; font-weight: 800; color: #0d2038;");
    hero_text_layout->addWidget(hero_title);

    QLabel* hero_subtitle = new QLabel(
        "Track vehicle readiness, launch simulations, and review mission telemetry from one calm, card-based control surface.");
    hero_subtitle->setObjectName("MutedText");
    hero_subtitle->setWordWrap(true);
    hero_subtitle->setStyleSheet("font-size: 14px;");
    hero_text_layout->addWidget(hero_subtitle);

    QHBoxLayout* hero_badges = new QHBoxLayout;
    hero_badges->setSpacing(10);

    QLabel* ready_badge = new QLabel("2 vehicles staged");
    ready_badge->setObjectName("StatusPill");
    hero_badges->addWidget(ready_badge);

    QLabel* ros_badge = new QLabel("ROS2 link online");
    ros_badge->setObjectName("StatusPill");
    hero_badges->addWidget(ros_badge);
    hero_badges->addStretch();

    hero_text_layout->addLayout(hero_badges);
    hero_text_layout->addStretch();
    hero_layout->addLayout(hero_text_layout, 3);

    QFrame* spotlight_card = new QFrame;
    spotlight_card->setStyleSheet("background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #16335f, stop:1 #2f80ff); border-radius: 20px;");
    QVBoxLayout* spotlight_layout = new QVBoxLayout(spotlight_card);
    spotlight_layout->setContentsMargins(22, 22, 22, 22);
    spotlight_layout->setSpacing(10);

    QLabel* spotlight_title = new QLabel("Today");
    spotlight_title->setStyleSheet("color: rgba(255,255,255,0.75); font-size: 12px; font-weight: 700; text-transform: uppercase;");
    spotlight_layout->addWidget(spotlight_title);

    QLabel* spotlight_value = new QLabel("84%");
    spotlight_value->setStyleSheet("color: white; font-size: 44px; font-weight: 800;");
    spotlight_layout->addWidget(spotlight_value);

    QLabel* spotlight_caption = new QLabel("Average fleet battery health");
    spotlight_caption->setStyleSheet("color: rgba(255,255,255,0.85); font-size: 13px;");
    spotlight_caption->setWordWrap(true);
    spotlight_layout->addWidget(spotlight_caption);
    spotlight_layout->addStretch();
    hero_layout->addWidget(spotlight_card, 1);

    content_layout->addWidget(hero_card);

    QHBoxLayout* metrics_layout = new QHBoxLayout;
    metrics_layout->setSpacing(18);

    const QList<QPair<QString, QString>> metrics = {
        {"02", "Active UAVs"},
        {"01", "Simulation cluster"},
        {"03", "Queued waypoints"},
        {"Live", "Telemetry heartbeat"}
    };

    for (const auto& metric : metrics) {
        QFrame* metric_card = new QFrame;
        metric_card->setObjectName("MetricCard");
        QVBoxLayout* metric_layout = new QVBoxLayout(metric_card);
        metric_layout->setContentsMargins(22, 18, 22, 18);
        metric_layout->setSpacing(6);

        QLabel* value = new QLabel(metric.first);
        value->setObjectName("MetricValue");
        metric_layout->addWidget(value);

        QLabel* caption = new QLabel(metric.second);
        caption->setObjectName("MetricCaption");
        metric_layout->addWidget(caption);

        metrics_layout->addWidget(metric_card);
    }

    content_layout->addLayout(metrics_layout);
    content_layout->addWidget(createRightPanel(), 1);
}

QWidget* UAVFleetGUI::createLeftPanel() {
    QFrame* panel = new QFrame;
    panel->setObjectName("SidebarCard");

    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setSpacing(20);

    QLabel* eyebrow = new QLabel("UAV fleet");
    eyebrow->setObjectName("AppEyebrow");
    layout->addWidget(eyebrow);

    QLabel* title = new QLabel("Cloud Fleet");
    title->setObjectName("SidebarTitle");
    layout->addWidget(title);

    QLabel* subtitle = new QLabel("Mission care dashboard inspired by the calmer, card-led feel of CloudCare.");
    subtitle->setObjectName("SidebarSubtitle");
    subtitle->setWordWrap(true);
    layout->addWidget(subtitle);

    QLabel* status = new QLabel("System healthy");
    status->setObjectName("StatusPill");
    layout->addWidget(status);

    QLabel* fleet_title = new QLabel("Vehicle roster");
    fleet_title->setObjectName("SectionTitle");
    layout->addWidget(fleet_title);

    uav_list_ = new QListWidget;
    uav_list_->addItem("UAV 1  |  Ready");
    uav_list_->addItem("UAV 2  |  Standby");
    uav_list_->setCurrentRow(0);
    layout->addWidget(uav_list_, 1);

    QLabel* table_title = new QLabel("Readiness snapshot");
    table_title->setObjectName("SectionTitle");
    layout->addWidget(table_title);

    status_table_ = new QTableWidget;
    status_table_->setColumnCount(3);
    status_table_->setHorizontalHeaderLabels({"UAV", "Status", "Battery"});
    status_table_->horizontalHeader()->setStretchLastSection(true);
    status_table_->verticalHeader()->setVisible(false);
    status_table_->setShowGrid(false);
    status_table_->setSelectionMode(QAbstractItemView::NoSelection);
    status_table_->setFocusPolicy(Qt::NoFocus);
    status_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    status_table_->setRowCount(2);
    status_table_->setItem(0, 0, new QTableWidgetItem("01"));
    status_table_->setItem(0, 1, new QTableWidgetItem("Ready"));
    status_table_->setItem(0, 2, new QTableWidgetItem("88%"));
    status_table_->setItem(1, 0, new QTableWidgetItem("02"));
    status_table_->setItem(1, 1, new QTableWidgetItem("Standby"));
    status_table_->setItem(1, 2, new QTableWidgetItem("80%"));
    layout->addWidget(status_table_);

    QLabel* actions_title = new QLabel("Quick actions");
    actions_title->setObjectName("SectionTitle");
    layout->addWidget(actions_title);

    start_sim_btn_ = new QPushButton("Start PX4 Simulation");
    start_sim_btn_->setObjectName("PrimaryButton");
    connect(start_sim_btn_, &QPushButton::clicked, this, &UAVFleetGUI::startSimulation);
    layout->addWidget(start_sim_btn_);

    arm_btn_ = new QPushButton("Arm Selected UAV");
    connect(arm_btn_, &QPushButton::clicked, this, &UAVFleetGUI::armUAVs);
    layout->addWidget(arm_btn_);

    setpoint_btn_ = new QPushButton("Send Setpoints");
    connect(setpoint_btn_, &QPushButton::clicked, this, &UAVFleetGUI::sendSetpoints);
    layout->addWidget(setpoint_btn_);

    return panel;
}

QTabWidget* UAVFleetGUI::createRightPanel() {
    QTabWidget* panel = new QTabWidget;
    panel->addTab(createDashboardTab(), "Overview");
    panel->addTab(createMissionTab(), "Missions");
    panel->addTab(createTelemetryTab(), "Telemetry");
    panel->addTab(createLogsTab(), "Ops Log");
    return panel;
}

QWidget* UAVFleetGUI::createDashboardTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);
    layout->setContentsMargins(0, 4, 0, 0);
    layout->setSpacing(18);

    QFrame* map_card = new QFrame;
    map_card->setObjectName("ContentCard");
    QVBoxLayout* map_layout = new QVBoxLayout(map_card);
    map_layout->setContentsMargins(24, 22, 24, 22);
    map_layout->setSpacing(14);

    QLabel* map_title = new QLabel("Operational airspace");
    map_title->setObjectName("SectionTitle");
    map_layout->addWidget(map_title);

    QLabel* map_subtitle = new QLabel("A shared staging view for launch corridors, rendezvous markers, and mission progress.");
    map_subtitle->setObjectName("MutedText");
    map_subtitle->setWordWrap(true);
    map_layout->addWidget(map_subtitle);

    QLabel* map_view = new QLabel("Map canvas\n\nWaypoint heatmap, geofence overlays, and UAV traces can land here next.");
    map_view->setAlignment(Qt::AlignCenter);
    map_view->setMinimumHeight(240);
    map_view->setStyleSheet(
        "background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #d8ecff, stop:1 #eef6ff);"
        "border: 1px dashed #8eb6e8; border-radius: 18px; color: #33537d; font-size: 16px; font-weight: 600;");
    map_layout->addWidget(map_view);
    layout->addWidget(map_card);

    QHBoxLayout* lower_layout = new QHBoxLayout;
    lower_layout->setSpacing(18);

    QFrame* mission_card = new QFrame;
    mission_card->setObjectName("ContentCard");
    QVBoxLayout* mission_layout = new QVBoxLayout(mission_card);
    mission_layout->setContentsMargins(22, 20, 22, 20);
    mission_layout->setSpacing(12);
    QLabel* mission_title = new QLabel("Mission pulse");
    mission_title->setObjectName("SectionTitle");
    mission_layout->addWidget(mission_title);
    mission_layout->addWidget(new QLabel("Survey Alpha queued for takeoff"));
    mission_layout->addWidget(new QLabel("Wind envelope: nominal"));
    mission_layout->addWidget(new QLabel("Failsafe policy: return-to-home"));
    mission_layout->addStretch();
    lower_layout->addWidget(mission_card, 1);

    QFrame* care_card = new QFrame;
    care_card->setObjectName("ContentCard");
    QVBoxLayout* care_layout = new QVBoxLayout(care_card);
    care_layout->setContentsMargins(22, 20, 22, 20);
    care_layout->setSpacing(12);
    QLabel* care_title = new QLabel("Operator notes");
    care_title->setObjectName("SectionTitle");
    care_layout->addWidget(care_title);
    QLabel* care_body = new QLabel(
        "Use this space the way CloudCare used contextual reassurance: key info first, no clutter, and one obvious next action.");
    care_body->setObjectName("MutedText");
    care_body->setWordWrap(true);
    care_layout->addWidget(care_body);
    QPushButton* mission_button = new QPushButton("Start Mission");
    mission_button->setObjectName("PrimaryButton");
    connect(mission_button, &QPushButton::clicked, this, &UAVFleetGUI::startMission);
    care_layout->addWidget(mission_button);
    care_layout->addStretch();
    lower_layout->addWidget(care_card, 1);

    layout->addLayout(lower_layout);
    return tab;
}

QWidget* UAVFleetGUI::createMissionTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);
    layout->setContentsMargins(0, 4, 0, 0);
    layout->setSpacing(18);

    QFrame* config_card = new QFrame;
    config_card->setObjectName("ContentCard");
    QVBoxLayout* config_card_layout = new QVBoxLayout(config_card);
    config_card_layout->setContentsMargins(24, 22, 24, 22);
    config_card_layout->setSpacing(16);

    QLabel* config_title = new QLabel("Mission configuration");
    config_title->setObjectName("SectionTitle");
    config_card_layout->addWidget(config_title);

    QLabel* config_hint = new QLabel("Shape a flight plan with the same structured, high-confidence rhythm as the CloudCare intake screens.");
    config_hint->setObjectName("MutedText");
    config_hint->setWordWrap(true);
    config_card_layout->addWidget(config_hint);

    QGridLayout* config_layout = new QGridLayout;
    config_layout->setHorizontalSpacing(14);
    config_layout->setVerticalSpacing(14);

    config_layout->addWidget(new QLabel("Mission type"), 0, 0);
    mission_type_ = new QComboBox;
    mission_type_->addItems({"Waypoint Navigation", "Area Survey", "Return to Home", "Custom"});
    config_layout->addWidget(mission_type_, 0, 1);

    config_layout->addWidget(new QLabel("Altitude (m)"), 1, 0);
    altitude_input_ = new QLineEdit("10");
    config_layout->addWidget(altitude_input_, 1, 1);

    config_layout->addWidget(new QLabel("Cruise speed (m/s)"), 2, 0);
    speed_input_ = new QLineEdit("2.5");
    config_layout->addWidget(speed_input_, 2, 1);

    config_card_layout->addLayout(config_layout);
    layout->addWidget(config_card);

    QFrame* waypoint_card = new QFrame;
    waypoint_card->setObjectName("ContentCard");
    QVBoxLayout* waypoint_layout = new QVBoxLayout(waypoint_card);
    waypoint_layout->setContentsMargins(24, 22, 24, 22);
    waypoint_layout->setSpacing(16);

    QLabel* waypoint_title = new QLabel("Waypoint planner");
    waypoint_title->setObjectName("SectionTitle");
    waypoint_layout->addWidget(waypoint_title);

    waypoint_table_ = new QTableWidget;
    waypoint_table_->setColumnCount(4);
    waypoint_table_->setHorizontalHeaderLabels({"X", "Y", "Z", "Action"});
    waypoint_table_->horizontalHeader()->setStretchLastSection(true);
    waypoint_table_->verticalHeader()->setVisible(false);
    waypoint_table_->setShowGrid(false);
    waypoint_layout->addWidget(waypoint_table_);

    QHBoxLayout* waypoint_actions = new QHBoxLayout;
    QPushButton* add_wp_btn = new QPushButton("Add Waypoint");
    add_wp_btn->setObjectName("PrimaryButton");
    connect(add_wp_btn, &QPushButton::clicked, this, &UAVFleetGUI::addWaypoint);
    waypoint_actions->addWidget(add_wp_btn);

    QPushButton* start_btn = new QPushButton("Launch Mission");
    connect(start_btn, &QPushButton::clicked, this, &UAVFleetGUI::startMission);
    waypoint_actions->addWidget(start_btn);
    waypoint_actions->addStretch();
    waypoint_layout->addLayout(waypoint_actions);

    layout->addWidget(waypoint_card, 1);
    return tab;
}

QWidget* UAVFleetGUI::createTelemetryTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);
    layout->setContentsMargins(0, 4, 0, 0);
    layout->setSpacing(18);

    QFrame* telemetry_card = new QFrame;
    telemetry_card->setObjectName("ContentCard");
    QVBoxLayout* telemetry_card_layout = new QVBoxLayout(telemetry_card);
    telemetry_card_layout->setContentsMargins(24, 22, 24, 22);
    telemetry_card_layout->setSpacing(18);

    QLabel* telemetry_title = new QLabel("Telemetry focus");
    telemetry_title->setObjectName("SectionTitle");
    telemetry_card_layout->addWidget(telemetry_title);

    QLabel* telemetry_copy = new QLabel("Readable, reassurance-first monitoring for whichever aircraft you have selected.");
    telemetry_copy->setObjectName("MutedText");
    telemetry_copy->setWordWrap(true);
    telemetry_card_layout->addWidget(telemetry_copy);

    QGridLayout* telemetry_layout = new QGridLayout;
    telemetry_layout->setHorizontalSpacing(14);
    telemetry_layout->setVerticalSpacing(14);

    auto addTelemetryCard = [&](const QString& title, QLabel*& value_label, int row, int col) {
        QFrame* stat = new QFrame;
        stat->setStyleSheet("background: #f8fbff; border: 1px solid #d7e2f0; border-radius: 18px;");
        QVBoxLayout* stat_layout = new QVBoxLayout(stat);
        stat_layout->setContentsMargins(18, 16, 18, 16);
        stat_layout->setSpacing(6);

        QLabel* stat_title = new QLabel(title);
        stat_title->setStyleSheet("color: #6b7a90; font-size: 12px; font-weight: 700; text-transform: uppercase;");
        stat_layout->addWidget(stat_title);

        value_label = new QLabel("N/A");
        value_label->setStyleSheet("color: #11233f; font-size: 24px; font-weight: 800;");
        stat_layout->addWidget(value_label);

        telemetry_layout->addWidget(stat, row, col);
    };

    addTelemetryCard("Selected UAV", selected_uav_telemetry_, 0, 0);
    addTelemetryCard("Position", position_label_, 0, 1);
    addTelemetryCard("Velocity", velocity_label_, 1, 0);
    addTelemetryCard("Battery", battery_label_, 1, 1);
    addTelemetryCard("GPS Fix", gps_label_, 2, 0);

    QFrame* status_story = new QFrame;
    status_story->setStyleSheet("background: #16335f; border-radius: 18px;");
    QVBoxLayout* story_layout = new QVBoxLayout(status_story);
    story_layout->setContentsMargins(18, 16, 18, 16);
    story_layout->setSpacing(8);

    QLabel* story_title = new QLabel("Operator summary");
    story_title->setStyleSheet("color: rgba(255,255,255,0.72); font-size: 12px; font-weight: 700; text-transform: uppercase;");
    story_layout->addWidget(story_title);

    QLabel* story_body = new QLabel("Telemetry cadence is stable. Selected aircraft remains inside expected survey envelope.");
    story_body->setWordWrap(true);
    story_body->setStyleSheet("color: white; font-size: 18px; font-weight: 700;");
    story_layout->addWidget(story_body);
    telemetry_layout->addWidget(status_story, 2, 1);

    telemetry_card_layout->addLayout(telemetry_layout);
    layout->addWidget(telemetry_card);
    return tab;
}

QWidget* UAVFleetGUI::createLogsTab() {
    QWidget* tab = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(tab);
    layout->setContentsMargins(0, 4, 0, 0);
    layout->setSpacing(18);

    QFrame* log_card = new QFrame;
    log_card->setObjectName("ContentCard");
    QVBoxLayout* log_layout = new QVBoxLayout(log_card);
    log_layout->setContentsMargins(24, 22, 24, 22);
    log_layout->setSpacing(16);

    QLabel* log_title = new QLabel("Operations log");
    log_title->setObjectName("SectionTitle");
    log_layout->addWidget(log_title);

    QLabel* log_copy = new QLabel("Simulation outputs, mission notes, and operator actions collect here in a calmer audit trail.");
    log_copy->setObjectName("MutedText");
    log_copy->setWordWrap(true);
    log_layout->addWidget(log_copy);

    output_text_ = new QTextEdit;
    output_text_->setReadOnly(true);
    output_text_->setPlaceholderText("Simulation output and mission notes will appear here...");
    log_layout->addWidget(output_text_, 1);

    QHBoxLayout* controls = new QHBoxLayout;
    QPushButton* clear_btn = new QPushButton("Clear Logs");
    connect(clear_btn, &QPushButton::clicked, this, &UAVFleetGUI::clearLogs);
    controls->addWidget(clear_btn);

    QPushButton* save_btn = new QPushButton("Save Logs");
    connect(save_btn, &QPushButton::clicked, this, &UAVFleetGUI::saveLogs);
    controls->addWidget(save_btn);
    controls->addStretch();
    log_layout->addLayout(controls);

    layout->addWidget(log_card);
    return tab;
}

void UAVFleetGUI::startSimulation() {
    const QString command = "cd ~/PX4-Autopilot && ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 2";
    sim_thread_ = new SimulationThread(command);
    connect(sim_thread_, &SimulationThread::outputReceived, this, &UAVFleetGUI::updateOutput);
    sim_thread_->start();
    updateOutput("Simulation launch requested for 2 PX4 vehicles.");
}

void UAVFleetGUI::armUAVs() {
    QListWidgetItem* selected = uav_list_->currentItem();
    if (selected) {
        const int uav_id = selected->text().split(" ").first().remove("UAV").toInt();
        qDebug() << "Arming UAV" << uav_id;
        updateOutput(QString("Arm command staged for UAV %1.").arg(uav_id));
    }
}

void UAVFleetGUI::sendSetpoints() {
    QListWidgetItem* selected = uav_list_->currentItem();
    if (selected) {
        const int uav_id = selected->text().split(" ").first().remove("UAV").toInt();
        qDebug() << "Sending setpoints to UAV" << uav_id;
        updateOutput(QString("Setpoints sent to UAV %1 at altitude %2 m.").arg(uav_id).arg(altitude_input_->text()));
    }
}

void UAVFleetGUI::addWaypoint() {
    const int row = waypoint_table_->rowCount();
    waypoint_table_->insertRow(row);
    waypoint_table_->setItem(row, 0, new QTableWidgetItem("0"));
    waypoint_table_->setItem(row, 1, new QTableWidgetItem("0"));
    waypoint_table_->setItem(row, 2, new QTableWidgetItem("10"));

    QPushButton* delete_btn = new QPushButton("Delete");
    connect(delete_btn, &QPushButton::clicked, [this, row]() { deleteWaypoint(row); });
    waypoint_table_->setCellWidget(row, 3, delete_btn);

    updateOutput(QString("Waypoint %1 added to the mission planner.").arg(row + 1));
}

void UAVFleetGUI::deleteWaypoint(int row) {
    waypoint_table_->removeRow(row);
    updateOutput(QString("Waypoint %1 removed.").arg(row + 1));
}

void UAVFleetGUI::startMission() {
    updateOutput(QString("Mission \"%1\" is ready for execution at %2 m altitude.")
        .arg(mission_type_ ? mission_type_->currentText() : "Waypoint Navigation")
        .arg(altitude_input_ ? altitude_input_->text() : "10"));
}

void UAVFleetGUI::clearLogs() {
    output_text_->clear();
}

void UAVFleetGUI::saveLogs() {
    updateOutput("Logs saved to operator archive.");
}

void UAVFleetGUI::updateTelemetry() {
    QListWidgetItem* selected = uav_list_->currentItem();
    if (!selected) {
        return;
    }

    selected_uav_telemetry_->setText(selected->text().split("|").first().trimmed());
    position_label_->setText("0.0, 0.0, 5.0");
    velocity_label_->setText(speed_input_ ? speed_input_->text() + " m/s" : "2.5 m/s");
    battery_label_->setText(selected->text().contains("Standby") ? "80%" : "88%");
    gps_label_->setText("3D Fix");
}

void UAVFleetGUI::updateOutput(const QString& text) {
    output_text_->append(text);
}
