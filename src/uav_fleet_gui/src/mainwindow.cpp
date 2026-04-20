#include "mainwindow.hpp"
#include "uav_control_node.hpp"

#include <QDebug>
#include <QFrame>
#include <QIcon>
#include <QHBoxLayout>
#include <QListView>
#include <QNetworkReply>
#include <QPainter>
#include <QPainterPath>
#include <QPointer>
#include <QProcess>
#include <QStyleFactory>
#include <QScrollArea>
#include <QtMath>
#include <QVBoxLayout>

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
    : QMainWindow(parent),
      node_(nullptr),
      sim_thread_(nullptr),
      uav_grid_container_(nullptr),
      uav_grid_layout_(nullptr),
      setup_uavs_btn_(nullptr),
      start_sim_btn_(nullptr),
      arm_btn_(nullptr),
      setpoint_btn_(nullptr),
      mission_type_(nullptr),
      altitude_input_(nullptr),
      speed_input_(nullptr),
      fleet_count_label_(nullptr),
      selected_uav_telemetry_(nullptr),
      position_label_(nullptr),
      velocity_label_(nullptr),
      battery_label_(nullptr),
      gps_label_(nullptr),
      output_text_(nullptr),
      telemetry_timer_(nullptr),
      map_network_manager_(new QNetworkAccessManager(this)),
      selected_uav_id_(1),
      uav_count_(10) {
    setupUI();

    telemetry_timer_ = new QTimer(this);
    connect(telemetry_timer_, &QTimer::timeout, this, &UAVFleetGUI::updateTelemetry);
    telemetry_timer_->start(1200);
}

UAVFleetGUI::~UAVFleetGUI() {
    if (sim_thread_ && sim_thread_->isRunning()) {
        sim_thread_->terminate();
        sim_thread_->wait();
    }
}

void UAVFleetGUI::setupUI() {
    setWindowTitle("UAV Fleet");
    resize(1600, 910);
    setMinimumSize(1280, 800);

    setStyleSheet(R"(
        QMainWindow {
            background: #14121d;
        }
        QWidget {
            background: transparent;
            color: #f8f8f2;
            font-family: "Noto Sans", "DejaVu Sans", sans-serif;
            font-size: 14px;
        }
        QFrame#TopBar {
            background: #17141f;
            border: 1px solid #242134;
            border-radius: 18px;
        }
        QFrame#CardPanel, QFrame#DetailPanel, QFrame#LogPanel, QFrame#UAVCard {
            background: #1b1827;
            border: 1px solid #2a2638;
            border-radius: 18px;
        }
        QFrame#UAVCard[selected="true"] {
            border: 1px solid #bd93f9;
            background: #211d31;
        }
        QLabel#AppTitle {
            color: #f8f8f2;
            font-size: 28px;
            font-weight: 800;
        }
        QLabel#MutedText {
            color: #b8b4ca;
            font-size: 13px;
        }
        QLabel#SectionTitle {
            color: #f8f8f2;
            font-size: 18px;
            font-weight: 700;
        }
        QLabel#StatusChipReady, QLabel#StatusChipIdle, QLabel#StatusChipLink {
            border-radius: 12px;
            padding: 4px 10px;
            font-size: 12px;
            font-weight: 700;
        }
        QLabel#StatusChipReady {
            background: rgba(80, 250, 123, 0.16);
            color: #50fa7b;
            border: 1px solid rgba(80, 250, 123, 0.35);
        }
        QLabel#StatusChipIdle {
            background: rgba(255, 184, 108, 0.16);
            color: #ffb86c;
            border: 1px solid rgba(255, 184, 108, 0.35);
        }
        QLabel#StatusChipLink {
            background: rgba(139, 233, 253, 0.14);
            color: #8be9fd;
            border: 1px solid rgba(139, 233, 253, 0.30);
        }
        QLabel#MetricValue {
            color: #f8f8f2;
            font-size: 22px;
            font-weight: 800;
        }
        QLabel#MetricLabel {
            color: #9f9ab3;
            font-size: 12px;
            font-weight: 700;
        }
        QLabel#UAVTitle {
            color: #f8f8f2;
            font-size: 15px;
            font-weight: 800;
        }
        QLabel#UAVSubtle {
            color: #8f89a3;
            font-size: 12px;
        }
        QPushButton#MapThumb {
            background: #1f1b2b;
            border: 1px solid #36314a;
            border-radius: 14px;
            color: rgba(248, 248, 242, 0.90);
            font-size: 16px;
            font-weight: 700;
        }
        QPushButton#MapThumb:hover {
            background: #262136;
            border: 1px solid #4f4670;
        }
        QLabel#DataPill {
            background: #262234;
            border: 1px solid #322d45;
            border-radius: 10px;
            padding: 6px 10px;
            color: #d7d4e4;
        }
        QPushButton {
            background: #2a2638;
            border: 1px solid #3a3550;
            border-radius: 12px;
            color: #f8f8f2;
            padding: 10px 16px;
            font-weight: 700;
            outline: none;
        }
        QPushButton:hover {
            background: #343048;
        }
        QPushButton:focus {
            border: 1px solid #6272a4;
        }
        QPushButton#PrimaryButton {
            background: #6272a4;
            border: 1px solid #7587bb;
        }
        QPushButton#PrimaryButton:hover {
            background: #7082b8;
        }
        QPushButton#MissionButton {
            background: #6f83b6;
            border: none;
            color: #f8f8f2;
            padding: 10px 14px;
        }
        QPushButton#MissionButton:hover {
            background: #7b8fc1;
        }
        QLineEdit, QComboBox, QTextEdit {
            background: #211d2d;
            border: 1px solid #332e45;
            border-radius: 12px;
            padding: 10px 12px;
            color: #f8f8f2;
            selection-background-color: #6272a4;
            selection-color: #f8f8f2;
        }
        QLineEdit:focus, QComboBox:focus, QTextEdit:focus {
            border: 1px solid #6272a4;
        }
        QComboBox::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 34px;
            border-left: 1px solid #3c3651;
            background: #2a2638;
            border-top-right-radius: 12px;
            border-bottom-right-radius: 12px;
        }
        QComboBox::down-arrow {
            image: none;
            width: 12px;
            height: 12px;
            margin-right: 10px;
        }
        QComboBox QAbstractItemView {
            background: #211d2d;
            color: #f8f8f2;
            border: 1px solid #3a3550;
            selection-background-color: #6272a4;
            selection-color: #f8f8f2;
            outline: none;
        }
        QComboBox QAbstractItemView::item {
            min-height: 28px;
            padding: 6px 10px;
        }
        QScrollArea {
            border: none;
            background: transparent;
        }
        QScrollBar:vertical {
            background: #15131f;
            width: 12px;
            margin: 2px;
        }
        QScrollBar::handle:vertical {
            background: #3d3752;
            border-radius: 6px;
            min-height: 28px;
        }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical,
        QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
            background: none;
            border: none;
        }
    )");

    QWidget* central_widget = new QWidget;
    setCentralWidget(central_widget);

    QVBoxLayout* root_layout = new QVBoxLayout(central_widget);
    root_layout->setContentsMargins(20, 18, 20, 18);
    root_layout->setSpacing(16);

    QFrame* top_bar = new QFrame;
    top_bar->setObjectName("TopBar");
    QHBoxLayout* top_layout = new QHBoxLayout(top_bar);
    top_layout->setContentsMargins(20, 12, 20, 12);
    top_layout->setSpacing(16);

    QVBoxLayout* title_layout = new QVBoxLayout;
    title_layout->setSpacing(2);

    QLabel* title = new QLabel("UAVs Web Management System");
    title->setObjectName("AppTitle");
    title->setStyleSheet("font-size: 23px; font-weight: 800;");
    title_layout->addWidget(title);

    QLabel* subtitle = new QLabel("Monitor and control your drone");
    subtitle->setObjectName("MutedText");
    subtitle->setStyleSheet("font-size: 12px;");
    title_layout->addWidget(subtitle);
    top_layout->addLayout(title_layout, 1);

    fleet_count_label_ = new QLabel("10 UAVs online");
    fleet_count_label_->setObjectName("StatusChipLink");
    top_layout->addWidget(fleet_count_label_);

    setup_uavs_btn_ = new QPushButton("Setup UAVs");
    setup_uavs_btn_->setObjectName("PrimaryButton");
    connect(setup_uavs_btn_, &QPushButton::clicked, this, &UAVFleetGUI::setupUAVs);
    top_layout->addWidget(setup_uavs_btn_);

    start_sim_btn_ = new QPushButton("Start PX4 Simulation");
    connect(start_sim_btn_, &QPushButton::clicked, this, &UAVFleetGUI::startSimulation);
    top_layout->addWidget(start_sim_btn_);

    root_layout->addWidget(top_bar);

    QHBoxLayout* content_layout = new QHBoxLayout;
    content_layout->setSpacing(16);
    root_layout->addLayout(content_layout, 1);

    QFrame* grid_panel = new QFrame;
    grid_panel->setObjectName("CardPanel");
    QVBoxLayout* grid_panel_layout = new QVBoxLayout(grid_panel);
    grid_panel_layout->setContentsMargins(18, 18, 18, 18);
    grid_panel_layout->setSpacing(14);

    QLabel* grid_title = new QLabel("Fleet board");
    grid_title->setObjectName("SectionTitle");
    grid_panel_layout->addWidget(grid_title);

    QScrollArea* fleet_scroll = new QScrollArea;
    fleet_scroll->setWidgetResizable(true);

    uav_grid_container_ = new QWidget;
    uav_grid_layout_ = new QGridLayout(uav_grid_container_);
    uav_grid_layout_->setContentsMargins(0, 0, 0, 0);
    uav_grid_layout_->setHorizontalSpacing(14);
    uav_grid_layout_->setVerticalSpacing(14);
    fleet_scroll->setWidget(uav_grid_container_);

    grid_panel_layout->addWidget(fleet_scroll, 1);
    content_layout->addWidget(grid_panel, 4);

    QVBoxLayout* side_layout = new QVBoxLayout;
    side_layout->setSpacing(16);
    content_layout->addLayout(side_layout, 2);

    QFrame* detail_panel = new QFrame;
    detail_panel->setObjectName("DetailPanel");
    QVBoxLayout* detail_layout = new QVBoxLayout(detail_panel);
    detail_layout->setContentsMargins(18, 18, 18, 18);
    detail_layout->setSpacing(14);

    QLabel* detail_title = new QLabel("Selected UAV");
    detail_title->setObjectName("SectionTitle");
    detail_layout->addWidget(detail_title);

    selected_uav_telemetry_ = new QLabel("Drone 1");
    selected_uav_telemetry_->setStyleSheet("font-size: 24px; font-weight: 800; color: #bd93f9;");
    detail_layout->addWidget(selected_uav_telemetry_);

    QGridLayout* metrics_layout = new QGridLayout;
    metrics_layout->setHorizontalSpacing(12);
    metrics_layout->setVerticalSpacing(12);

    auto addMetric = [&](const QString& title_text, QLabel*& value_label, int row, int col) {
        QFrame* metric = new QFrame;
        metric->setObjectName("DetailPanel");
        metric->setStyleSheet("QFrame { background: #211d2d; border: 1px solid #332e45; border-radius: 14px; }");
        QVBoxLayout* metric_layout = new QVBoxLayout(metric);
        metric_layout->setContentsMargins(14, 12, 14, 12);
        metric_layout->setSpacing(6);

        QLabel* title_label = new QLabel(title_text);
        title_label->setObjectName("MetricLabel");
        metric_layout->addWidget(title_label);

        value_label = new QLabel("N/A");
        value_label->setObjectName("MetricValue");
        metric_layout->addWidget(value_label);

        metrics_layout->addWidget(metric, row, col);
    };

    addMetric("Position", position_label_, 0, 0);
    addMetric("Velocity", velocity_label_, 0, 1);
    addMetric("Battery", battery_label_, 1, 0);
    addMetric("GPS", gps_label_, 1, 1);
    detail_layout->addLayout(metrics_layout);

    QLabel* mission_label = new QLabel("Mission Type");
    mission_label->setObjectName("MutedText");
    detail_layout->addWidget(mission_label);
    mission_type_ = new QComboBox;
    mission_type_->addItems({"Waypoint Navigation", "Area Survey", "Return to Home", "Custom"});
    mission_type_->setView(new QListView(mission_type_));
    mission_type_->setStyle(QStyleFactory::create("Fusion"));
    mission_type_->view()->setStyle(QStyleFactory::create("Fusion"));
    detail_layout->addWidget(mission_type_);

    QLabel* altitude_label = new QLabel("Altitude (m)");
    altitude_label->setObjectName("MutedText");
    detail_layout->addWidget(altitude_label);
    altitude_input_ = new QLineEdit("20");
    detail_layout->addWidget(altitude_input_);

    QLabel* speed_label = new QLabel("Cruise Speed (m/s)");
    speed_label->setObjectName("MutedText");
    detail_layout->addWidget(speed_label);
    speed_input_ = new QLineEdit("2.5");
    detail_layout->addWidget(speed_input_);

    arm_btn_ = new QPushButton("Arm Selected UAV");
    connect(arm_btn_, &QPushButton::clicked, this, &UAVFleetGUI::armUAVs);
    detail_layout->addWidget(arm_btn_);

    setpoint_btn_ = new QPushButton("Send Setpoints");
    connect(setpoint_btn_, &QPushButton::clicked, this, &UAVFleetGUI::sendSetpoints);
    detail_layout->addWidget(setpoint_btn_);

    QPushButton* mission_btn = new QPushButton("Open Mission");
    mission_btn->setObjectName("PrimaryButton");
    connect(mission_btn, &QPushButton::clicked, this, &UAVFleetGUI::startMission);
    detail_layout->addWidget(mission_btn);
    side_layout->addWidget(detail_panel);

    QFrame* log_panel = new QFrame;
    log_panel->setObjectName("LogPanel");
    QVBoxLayout* log_layout = new QVBoxLayout(log_panel);
    log_layout->setContentsMargins(18, 18, 18, 18);
    log_layout->setSpacing(12);

    QLabel* log_title = new QLabel("Operations Log");
    log_title->setObjectName("SectionTitle");
    log_layout->addWidget(log_title);

    output_text_ = new QTextEdit;
    output_text_->setReadOnly(true);
    output_text_->setMinimumHeight(220);
    output_text_->setPlaceholderText("Mission actions and PX4 output will appear here...");
    log_layout->addWidget(output_text_, 1);

    QHBoxLayout* log_actions = new QHBoxLayout;
    QPushButton* clear_btn = new QPushButton("Clear");
    connect(clear_btn, &QPushButton::clicked, this, &UAVFleetGUI::clearLogs);
    log_actions->addWidget(clear_btn);

    QPushButton* save_btn = new QPushButton("Save");
    connect(save_btn, &QPushButton::clicked, this, &UAVFleetGUI::saveLogs);
    log_actions->addWidget(save_btn);
    log_actions->addStretch();
    log_layout->addLayout(log_actions);
    side_layout->addWidget(log_panel, 1);

    populateUAVGrid();
    selectUAV(selected_uav_id_);
}

void UAVFleetGUI::populateUAVGrid() {
    while (QLayoutItem* item = uav_grid_layout_->takeAt(0)) {
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

    const int columns = 4;
    for (int i = 0; i < uav_count_; ++i) {
        QWidget* card = createUAVCard(i + 1);
        uav_grid_layout_->addWidget(card, i / columns, i % columns);
    }

    fleet_count_label_->setText(QString("%1 UAVs online").arg(uav_count_));
}

QWidget* UAVFleetGUI::createUAVCard(int uav_id) {
    QFrame* card = new QFrame;
    card->setObjectName("UAVCard");
    card->setProperty("selected", uav_id == selected_uav_id_);
    card->style()->unpolish(card);
    card->style()->polish(card);

    QVBoxLayout* card_layout = new QVBoxLayout(card);
    card_layout->setContentsMargins(12, 12, 12, 12);
    card_layout->setSpacing(10);

    QHBoxLayout* title_layout = new QHBoxLayout;
    title_layout->setSpacing(8);

    QLabel* title = new QLabel(QString("Drone %1").arg(uav_id));
    title->setObjectName("UAVTitle");
    title_layout->addWidget(title);

    QLabel* sub = new QLabel(QString("uav%1").arg(uav_id));
    sub->setObjectName("UAVSubtle");
    title_layout->addWidget(sub);
    title_layout->addStretch();

    QPushButton* mission_button = new QPushButton("Open Mission");
    mission_button->setObjectName("MissionButton");
    connect(mission_button, &QPushButton::clicked, this, [this, uav_id]() {
        selectUAV(uav_id);
        startMission();
    });
    title_layout->addWidget(mission_button);
    card_layout->addLayout(title_layout);

    QHBoxLayout* body_layout = new QHBoxLayout;
    body_layout->setSpacing(10);

    QPushButton* map_button = new QPushButton;
    map_button->setObjectName("MapThumb");
    map_button->setMinimumSize(240, 150);
    map_button->setCursor(Qt::PointingHandCursor);
    map_button->setIcon(QIcon(createMapPreview(uav_id)));
    map_button->setIconSize(QSize(236, 146));
    map_button->setText(QString(" "));
    requestMapPreview(map_button, uav_id);
    connect(map_button, &QPushButton::clicked, this, [this, uav_id]() { selectUAV(uav_id); });
    body_layout->addWidget(map_button, 1);

    QVBoxLayout* chip_layout = new QVBoxLayout;
    chip_layout->setSpacing(8);

    QLabel* state_chip = new QLabel((uav_id % 3 == 0) ? "ready" : "idle");
    state_chip->setObjectName((uav_id % 3 == 0) ? "StatusChipReady" : "StatusChipIdle");
    chip_layout->addWidget(state_chip);

    QLabel* mode_chip = new QLabel((uav_id == selected_uav_id_) ? "selected" : "mission");
    mode_chip->setObjectName("StatusChipLink");
    chip_layout->addWidget(mode_chip);

    QLabel* link_chip = new QLabel((uav_id % 2 == 0) ? "link ok" : "standby");
    link_chip->setObjectName("StatusChipIdle");
    chip_layout->addWidget(link_chip);
    chip_layout->addStretch();
    body_layout->addLayout(chip_layout);

    card_layout->addLayout(body_layout);

    QHBoxLayout* data_layout = new QHBoxLayout;
    data_layout->setSpacing(8);

    const QPointF coordinate = uavCoordinate(uav_id);
    const QString lat_value = QString::number(coordinate.x(), 'f', 4);
    const QString lon_value = QString::number(coordinate.y(), 'f', 4);

    QLabel* lat = new QLabel(QString("Lat\n%1").arg(lat_value));
    lat->setObjectName("DataPill");
    data_layout->addWidget(lat);

    QLabel* lon = new QLabel(QString("Lon\n%1").arg(lon_value));
    lon->setObjectName("DataPill");
    data_layout->addWidget(lon);

    QLabel* alt = new QLabel(QString("Alt\n%1").arg(18 + (uav_id % 5) * 2));
    alt->setObjectName("DataPill");
    data_layout->addWidget(alt);

    card_layout->addLayout(data_layout);
    return card;
}

QPixmap UAVFleetGUI::createMapPreview(int uav_id) const {
    QPixmap pixmap(236, 146);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QPainterPath frame_path;
    frame_path.addRoundedRect(QRectF(0, 0, 236, 146), 14, 14);
    painter.fillPath(frame_path, QColor("#1b2337"));
    painter.setClipPath(frame_path);

    QLinearGradient ocean(0, 0, 236, 146);
    ocean.setColorAt(0.0, QColor("#203a63"));
    ocean.setColorAt(0.45, QColor("#216e83"));
    ocean.setColorAt(1.0, QColor("#162447"));
    painter.fillRect(QRect(0, 0, 236, 146), ocean);

    painter.setPen(QPen(QColor(255, 255, 255, 24), 1));
    for (int x = 18; x < 236; x += 34) {
        painter.drawLine(x, 0, x, 146);
    }
    for (int y = 16; y < 146; y += 28) {
        painter.drawLine(0, y, 236, y);
    }

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#6f8f4e"));
    painter.drawEllipse(QRectF(14, 18, 78, 55));
    painter.drawEllipse(QRectF(54, 28, 66, 64));
    painter.drawEllipse(QRectF(132, 26, 82, 56));
    painter.drawEllipse(QRectF(150, 68, 56, 34));
    painter.drawEllipse(QRectF(96, 84, 64, 32));
    painter.drawEllipse(QRectF(172, 18, 28, 18));

    painter.setBrush(QColor(255, 255, 255, 30));
    painter.drawEllipse(QRectF(24, 28, 38, 18));
    painter.drawEllipse(QRectF(154, 34, 44, 16));

    painter.setPen(QPen(QColor("#ff79c6"), 2.4));
    const QPointF start(28 + ((uav_id * 17) % 48), 92 - ((uav_id * 7) % 20));
    const QPointF mid(104 + ((uav_id * 11) % 24), 60 + ((uav_id * 5) % 24));
    const QPointF end(188 - ((uav_id * 9) % 40), 32 + ((uav_id * 13) % 52));
    QPainterPath route(start);
    route.quadTo(QPointF(122, 104), mid);
    route.quadTo(QPointF(172, 84), end);
    painter.drawPath(route);

    painter.setBrush(QColor("#50fa7b"));
    painter.setPen(QPen(QColor("#0b0b10"), 1.5));
    painter.drawEllipse(start, 4.5, 4.5);

    painter.setBrush(QColor("#ffb86c"));
    painter.drawEllipse(mid, 4.5, 4.5);

    painter.setBrush(QColor("#8be9fd"));
    painter.drawEllipse(end, 5.0, 5.0);

    painter.setPen(QColor(248, 248, 242, 230));
    painter.setFont(QFont("Noto Sans", 10, QFont::Bold));
    painter.drawText(QRectF(12, 10, 140, 20), QString("Mission Grid %1").arg(uav_id));

    painter.end();
    return pixmap;
}

QPointF UAVFleetGUI::uavCoordinate(int uav_id) const {
    return QPointF(22.45 + (uav_id * 0.03), 94.91 + (uav_id * 0.02));
}

void UAVFleetGUI::requestMapPreview(QPushButton* map_button, int uav_id) {
    if (!map_button) {
        return;
    }

    if (map_preview_cache_.contains(uav_id)) {
        map_button->setIcon(QIcon(map_preview_cache_.value(uav_id)));
        return;
    }

    const QPointF coordinate = uavCoordinate(uav_id);
    const double lat_rad = qDegreesToRadians(coordinate.x());
    const int zoom = 8;
    const double scale = static_cast<double>(1 << zoom);
    const int x_tile = static_cast<int>((coordinate.y() + 180.0) / 360.0 * scale);
    const int y_tile = static_cast<int>((1.0 - std::log(std::tan(lat_rad) + (1.0 / std::cos(lat_rad))) / M_PI) / 2.0 * scale);
    const QUrl request_url(QString(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%1/%2/%3")
        .arg(zoom)
        .arg(y_tile)
        .arg(x_tile));

    QNetworkRequest request(request_url);
    request.setRawHeader("User-Agent", "uav_fleet_gui/0.0.1");

    QNetworkReply* reply = map_network_manager_->get(request);
    QPointer<QPushButton> safe_button(map_button);
    connect(reply, &QNetworkReply::finished, this, [this, reply, safe_button, uav_id]() {
        reply->deleteLater();

        if (reply->error() != QNetworkReply::NoError) {
            return;
        }

        QPixmap tile;
        if (!tile.loadFromData(reply->readAll())) {
            return;
        }

        QPixmap preview = tile.scaled(236, 146, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation);

        QPainter painter(&preview);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setBrush(QColor("#ff79c6"));
        painter.setPen(QPen(QColor("#1a1625"), 1.5));
        painter.drawEllipse(QPointF(preview.width() / 2.0, preview.height() / 2.0), 6.0, 6.0);
        painter.setPen(QColor(248, 248, 242, 225));
        painter.setFont(QFont("Noto Sans", 10, QFont::Bold));
        painter.drawText(QRect(10, 8, 140, 20), QString("UAV %1").arg(uav_id));
        painter.setFont(QFont("Noto Sans", 9));
        painter.drawText(QRect(10, preview.height() - 22, 170, 16), "ArcGIS World Imagery");
        painter.end();

        map_preview_cache_.insert(uav_id, preview);
        if (safe_button) {
            safe_button->setIcon(QIcon(preview));
        }
    });
}

void UAVFleetGUI::selectUAV(int uav_id) {
    selected_uav_id_ = uav_id;
    selected_uav_telemetry_->setText(QString("Drone %1").arg(uav_id));
    populateUAVGrid();
    updateTelemetry();
    updateOutput(QString("UAV %1 is now selected.").arg(uav_id));
}

void UAVFleetGUI::startSimulation() {
    const QString command = "cd ~/PX4-Autopilot && ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh";
    sim_thread_ = new SimulationThread(command);
    connect(sim_thread_, &SimulationThread::outputReceived, this, &UAVFleetGUI::updateOutput);
    sim_thread_->start();
    updateOutput("PX4 simulation launch requested.");
}

void UAVFleetGUI::armUAVs() {
    qDebug() << "Arming UAV" << selected_uav_id_;
    updateOutput(QString("Arm command sent to UAV %1.").arg(selected_uav_id_));
}

void UAVFleetGUI::sendSetpoints() {
    qDebug() << "Sending setpoints to UAV" << selected_uav_id_;
    updateOutput(QString("Setpoints pushed to UAV %1 at %2 m altitude and %3 m/s.")
        .arg(selected_uav_id_)
        .arg(altitude_input_->text())
        .arg(speed_input_->text()));
}

void UAVFleetGUI::setupUAVs() {
    ++uav_count_;
    populateUAVGrid();
    selectUAV(uav_count_);
    updateOutput(QString("Setup UAVs created UAV %1 and added it to the fleet wall.").arg(uav_count_));
}

void UAVFleetGUI::startMission() {
    updateOutput(QString("Mission opened for UAV %1 using %2 mode.")
        .arg(selected_uav_id_)
        .arg(mission_type_->currentText()));
}

void UAVFleetGUI::clearLogs() {
    output_text_->clear();
}

void UAVFleetGUI::saveLogs() {
    updateOutput("Logs saved from the operations panel.");
}

void UAVFleetGUI::updateTelemetry() {
    const QPointF coordinate = uavCoordinate(selected_uav_id_);
    position_label_->setText(QString("%1, %2")
        .arg(QString::number(coordinate.x(), 'f', 4))
        .arg(QString::number(coordinate.y(), 'f', 4)));
    velocity_label_->setText(QString("%1 m/s").arg(speed_input_->text()));
    battery_label_->setText(QString("%1%").arg(92 - ((selected_uav_id_ * 3) % 20)));
    gps_label_->setText((selected_uav_id_ % 2 == 0) ? "3D Fix" : "RTK");
}

void UAVFleetGUI::updateOutput(const QString& text) {
    if (!output_text_) {
        return;
    }
    output_text_->append(text);
}
