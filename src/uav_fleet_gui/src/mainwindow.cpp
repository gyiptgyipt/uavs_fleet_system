#include "mainwindow.hpp"
#include "gis_map_widget.hpp"
#include "uav_control_node.hpp"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QCheckBox>
#include <QDialog>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QListView>
#include <QNetworkReply>
#include <QPainter>
#include <QPainterPath>
#include <QPointer>
#include <QProcess>
#include <QSlider>
#include <QSplitter>
#include <QStyleFactory>
#include <QtMath>
#include <QVBoxLayout>
#include <algorithm>

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
      ui_(new Ui::UAVFleetGUI),
      interactive_map_(nullptr),
      node_(nullptr),
      sim_thread_(nullptr),
      uav_grid_container_(nullptr),
      uav_grid_layout_(nullptr),
      telemetry_timer_(nullptr),
      map_network_manager_(new QNetworkAccessManager(this)),
      stack_widget_(nullptr),
      main_page_(nullptr),
      mission_page_(nullptr),
      mission_title_label_(nullptr),
      mission_map_(nullptr),
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
    delete ui_;
}

void UAVFleetGUI::setupUI() {
    ui_->setupUi(this);

    QWidget* original_central = centralWidget();
    if (original_central) {
        original_central->setParent(nullptr);
    }

    stack_widget_ = new QStackedWidget(this);
    if (original_central) {
        stack_widget_->addWidget(original_central);
    }

    QWidget* root_container = new QWidget(this);
    QVBoxLayout* root_layout = new QVBoxLayout(root_container);
    root_layout->setContentsMargins(0, 0, 0, 0);
    root_layout->addWidget(stack_widget_);
    setCentralWidget(root_container);

    main_page_ = original_central;

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
        QFrame#topBar {
            background: #17141f;
            border: 1px solid #242134;
            border-radius: 18px;
        }
        QFrame#gridPanel, QFrame#detailPanel, QFrame#logPanel, QFrame#UAVCard,
        QFrame#positionFrame, QFrame#velocityFrame, QFrame#batteryFrame, QFrame#gpsFrame {
            background: #1b1827;
            border: 1px solid #2a2638;
            border-radius: 18px;
        }
        QFrame#UAVCard[selected="true"] {
            border: 1px solid #bd93f9;
            background: #211d31;
        }
        QLabel#titleLabel {
            color: #f8f8f2;
            font-size: 23px;
            font-weight: 800;
        }
        QLabel#subtitleLabel, QLabel#missionTypeLabel, QLabel#altitudeLabel, QLabel#speedLabel,
        QLabel#positionTitleLabel, QLabel#velocityTitleLabel, QLabel#batteryTitleLabel, QLabel#gpsTitleLabel {
            color: #30275a;
            font-size: 12px;
            font-weight: 600;
        }
        QLabel#fleetBoardLabel, QLabel#selectedUavTitleLabel, QLabel#operationsLogLabel {
            color: #f8f8f2;
            font-size: 18px;
            font-weight: 700;
        }
        QLabel#fleetCountLabel {
            border-radius: 12px;
            padding: 4px 10px;
            font-size: 12px;
            font-weight: 700;
            background: rgba(139, 233, 253, 0.14);
            color: #8be9fd;
            border: 1px solid rgba(139, 233, 253, 0.30);
        }
        QLabel#selectedUavValueLabel {
            font-size: 24px;
            font-weight: 800;
            color: #bd93f9;
        }
        QLabel#positionValueLabel, QLabel#velocityValueLabel, QLabel#batteryValueLabel, QLabel#gpsValueLabel {
            color: #f8f8f2;
            font-size: 22px;
            font-weight: 800;
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
        QLabel#DataPill {
            background: #262234;
            border: 1px solid #322d45;
            border-radius: 10px;
            padding: 6px 10px;
            color: #d7d4e4;
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
        QPushButton#addUavsButton, QPushButton#openMissionButton {
            background: #6272a4;
            border: 1px solid #7587bb;
        }
        QPushButton#addUavsButton:hover, QPushButton#openMissionButton:hover {
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
        QFrame#MapThumb {
            background: #1f1b2b;
            border: 1px solid #36314a;
            border-radius: 14px;
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
        QSplitter::handle {
            background: #1a1625;
        }
        QSplitter::handle:horizontal {
            width: 10px;
            margin: 0 4px;
            border-radius: 4px;
            border: 1px solid #2f2941;
        }
        QSplitter::handle:vertical {
            height: 10px;
            margin: 4px 0;
            border-radius: 4px;
            border: 1px solid #2f2941;
        }
    )");

    ui_->missionTypeCombo->addItems({"Waypoint Navigation", "Area Survey", "Return to Home", "Custom"});
    ui_->missionTypeCombo->setView(new QListView(ui_->missionTypeCombo));
    ui_->missionTypeCombo->setStyle(QStyleFactory::create("Fusion"));
    ui_->missionTypeCombo->view()->setStyle(QStyleFactory::create("Fusion"));
    ui_->missionTypeCombo->view()->setStyleSheet(R"(
        QListView {
            background: #211d2d;
            color: #f8f8f2;
            border: 1px solid #3a3550 !important;
            outline: none;
            margin: 0px;
            padding: 0px;
        }
        QListView::item {
            min-height: 28px;
            padding: 6px 10px;
            margin: 0px;
        }
        QListView::item:selected {
            background: #6272a4;
            color: #f8f8f2;
        }
    )");
    ui_->missionTypeCombo->view()->window()->setStyleSheet(R"(
        QWidget {
            background-color: #211d2d;
            border: 1px solid #3a3550;
        }
    )");

    ui_->mainSplitter->setStretchFactor(0, 4);
    ui_->mainSplitter->setStretchFactor(1, 2);
    ui_->mainSplitter->setSizes(QList<int>() << 1050 << 450);
    ui_->sideSplitter->setStretchFactor(0, 3);
    ui_->sideSplitter->setStretchFactor(1, 2);
    ui_->sideSplitter->setSizes(QList<int>() << 520 << 320);

    connect(ui_->addUavsButton, &QPushButton::clicked, this, &UAVFleetGUI::setupUAVs);
    connect(ui_->removeUavButton, &QPushButton::clicked, this, &UAVFleetGUI::removeUAV);
    connect(ui_->startSimButton, &QPushButton::clicked, this, &UAVFleetGUI::startSimulation);
    connect(ui_->armButton, &QPushButton::clicked, this, &UAVFleetGUI::armUAVs);
    connect(ui_->setpointButton, &QPushButton::clicked, this, &UAVFleetGUI::sendSetpoints);
    connect(ui_->openMissionButton, &QPushButton::clicked, this, &UAVFleetGUI::startMission);
    connect(ui_->clearLogButton, &QPushButton::clicked, this, &UAVFleetGUI::clearLogs);
    connect(ui_->saveLogButton, &QPushButton::clicked, this, &UAVFleetGUI::saveLogs);

    ui_->logTextEdit->setReadOnly(true);
    ui_->logTextEdit->setPlaceholderText("Mission actions and PX4 output will appear here...");

    uav_grid_container_ = ui_->fleetGridContainer;
    uav_grid_layout_ = ui_->fleetGridLayout;

    populateUAVGrid();
    selectUAV(selected_uav_id_);
}

void UAVFleetGUI::setupMissionPage() {
    mission_page_ = new QWidget();
    stack_widget_->addWidget(mission_page_);

    // Setup mission page
    QVBoxLayout* root_layout = new QVBoxLayout(mission_page_);
    root_layout->setContentsMargins(18, 18, 18, 18);
    root_layout->setSpacing(16);

    QFrame* header_frame = new QFrame(mission_page_);
    header_frame->setObjectName("detailPanel");
    QHBoxLayout* header_layout = new QHBoxLayout(header_frame);
    header_layout->setContentsMargins(14, 14, 14, 14);
    header_layout->setSpacing(14);

    QLabel* title_label = new QLabel(QString("Drone %1 Mission").arg(selected_uav_id_), header_frame);
    title_label->setStyleSheet("font-size: 24px; font-weight: 800; color: #f8f8f2;");
    header_layout->addWidget(title_label, 1);
    mission_title_label_ = title_label;

    QLabel* status_label = new QLabel("status loading...", header_frame);
    status_label->setStyleSheet("color: #8f89a3; font-size: 14px;");
    header_layout->addWidget(status_label);

    QPushButton* back_button = new QPushButton("Back", header_frame);
    back_button->setObjectName("MissionButton");
    connect(back_button, &QPushButton::clicked, this, [this]() {
        stack_widget_->setCurrentWidget(main_page_);
    });
    header_layout->addWidget(back_button);
    header_layout->addStretch();

    root_layout->addWidget(header_frame, 0);

    GISMapWidget* mission_map = new GISMapWidget(mission_page_);
    mission_map->setCenterCoordinate(uavCoordinate(selected_uav_id_).x(), uavCoordinate(selected_uav_id_).y());
    mission_map->setZoomLevel(12);
    mission_map->setMinimumHeight(440);
    root_layout->addWidget(mission_map, 1);
    mission_map_ = mission_map;

    QFrame* bottom_frame = new QFrame(mission_page_);
    bottom_frame->setObjectName("detailPanel");
    QHBoxLayout* bottom_layout = new QHBoxLayout(bottom_frame);
    bottom_layout->setContentsMargins(14, 14, 14, 14);
    bottom_layout->setSpacing(18);

    QPushButton* calibrate_button = new QPushButton("Calibrate Map", bottom_frame);
    calibrate_button->setObjectName("MissionButton");
    connect(calibrate_button, &QPushButton::clicked, this, [status_label]() {
        status_label->setText("Map calibrated");
    });
    bottom_layout->addWidget(calibrate_button, 0, Qt::AlignLeft);

    bottom_layout->addStretch();

    QVBoxLayout* right_side_layout = new QVBoxLayout;
    right_side_layout->setSpacing(14);

    QLabel* speed_label = new QLabel(QString("Speed (%1 m/s)").arg(ui_->speedLineEdit->text()), bottom_frame);
    QSlider* speed_slider = new QSlider(Qt::Horizontal, bottom_frame);
    speed_slider->setRange(0, 30);
    speed_slider->setValue(ui_->speedLineEdit->text().toInt());
    right_side_layout->addWidget(speed_label);
    right_side_layout->addWidget(speed_slider);

    QLabel* altitude_label = new QLabel(QString("Altitude (%1 m)").arg(ui_->altitudeLineEdit->text()), bottom_frame);
    QSlider* altitude_slider = new QSlider(Qt::Horizontal, bottom_frame);
    altitude_slider->setRange(0, 120);
    altitude_slider->setValue(ui_->altitudeLineEdit->text().toInt());
    right_side_layout->addWidget(altitude_label);
    right_side_layout->addWidget(altitude_slider);

    QCheckBox* manual_mode = new QCheckBox("Manual Mode", bottom_frame);
    manual_mode->setChecked(true);
    right_side_layout->addWidget(manual_mode);

    QVBoxLayout* buttons_layout = new QVBoxLayout;
    buttons_layout->setSpacing(12);
    QPushButton* arm_button = new QPushButton("Arm", bottom_frame);
    QPushButton* force_disarm_button = new QPushButton("Force Disarm", bottom_frame);
    QPushButton* land_button = new QPushButton("Land", bottom_frame);
    QPushButton* disarm_button = new QPushButton("Disarm", bottom_frame);
    buttons_layout->addWidget(arm_button);
    buttons_layout->addWidget(force_disarm_button);
    buttons_layout->addWidget(land_button);
    buttons_layout->addWidget(disarm_button);

    right_side_layout->addLayout(buttons_layout);

    bottom_layout->addLayout(right_side_layout, 0);

    root_layout->addWidget(bottom_frame, 0);

    connect(speed_slider, &QSlider::valueChanged, this, [speed_label](int value) {
        speed_label->setText(QString("Speed (%1 m/s)").arg(value));
    });
    connect(altitude_slider, &QSlider::valueChanged, this, [altitude_label](int value) {
        altitude_label->setText(QString("Altitude (%1 m)").arg(value));
    });
    connect(arm_button, &QPushButton::clicked, this, [this]() {
        updateOutput(QString("Arm command sent to UAV %1 from mission overlay.").arg(selected_uav_id_));
    });
    connect(disarm_button, &QPushButton::clicked, this, [this]() {
        updateOutput(QString("Disarm command sent to UAV %1 from mission overlay.").arg(selected_uav_id_));
    });
    connect(force_disarm_button, &QPushButton::clicked, this, [this]() {
        updateOutput(QString("Force disarm command sent to UAV %1 from mission overlay.").arg(selected_uav_id_));
    });
    connect(land_button, &QPushButton::clicked, this, [this]() {
        updateOutput(QString("Land command sent to UAV %1 from mission overlay.").arg(selected_uav_id_));
    });
}

void UAVFleetGUI::populateUAVGrid() {
    while (QLayoutItem* item = uav_grid_layout_->takeAt(0)) {
        if (item->widget()) {
            item->widget()->deleteLater();
        }
        delete item;
    }

    const int columns = 4;
    const int rows = (uav_count_ + columns - 1) / columns;
    for (int i = 0; i < uav_count_; ++i) {
        QWidget* card = createUAVCard(i + 1);
        uav_grid_layout_->addWidget(card, i / columns, i % columns);
    }
    for (int row = 0; row < rows; ++row) {
        uav_grid_layout_->setRowStretch(row, 1);
    }
    // Only set column stretch for columns that have UAVs
    const int actual_columns = std::min(columns, uav_count_);
    for (int col = 0; col < actual_columns; ++col) {
        uav_grid_layout_->setColumnStretch(col, 1);
    }
    // Add stretch to the right to prevent cards from expanding
    uav_grid_layout_->setColumnStretch(columns, 1);

    ui_->fleetCountLabel->setText(QString("%1 UAVs online").arg(uav_count_));
}

QWidget* UAVFleetGUI::createUAVCard(int uav_id) {
    QFrame* card = new QFrame;
    card->setObjectName("UAVCard");
    card->setProperty("selected", uav_id == selected_uav_id_);
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    card->setMaximumWidth(520);
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
    body_layout->setAlignment(Qt::AlignTop);

    QFrame* map_frame = new QFrame;
    map_frame->setObjectName("MapThumb");
    map_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QVBoxLayout* map_layout = new QVBoxLayout(map_frame);
    map_layout->setContentsMargins(0, 0, 0, 0);

    GISMapWidget* card_map = new GISMapWidget(map_frame);
    card_map->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    const QPointF coordinate = uavCoordinate(uav_id);
    card_map->setCenterCoordinate(coordinate.x(), coordinate.y());
    card_map->setZoomLevel(10);
    map_layout->addWidget(card_map);
    body_layout->addWidget(map_frame, 1);

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

    QLabel* lat = new QLabel(QString("Lat\n%1").arg(QString::number(coordinate.x(), 'f', 4)));
    lat->setObjectName("DataPill");
    data_layout->addWidget(lat);

    QLabel* lon = new QLabel(QString("Lon\n%1").arg(QString::number(coordinate.y(), 'f', 4)));
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
        painter.end();

        map_preview_cache_.insert(uav_id, preview);
        if (safe_button) {
            safe_button->setIcon(QIcon(preview));
        }
    });
}

void UAVFleetGUI::selectUAV(int uav_id) {
    selected_uav_id_ = uav_id;
    ui_->selectedUavValueLabel->setText(QString("Drone %1").arg(uav_id));
    populateUAVGrid();
    updateTelemetry();
    updateOutput(QString("UAV %1 is now selected.").arg(uav_id));
}

void UAVFleetGUI::openMissionDialog() {
    if (!mission_page_) {
        setupMissionPage();
    }
    mission_title_label_->setText(QString("Drone %1 Mission").arg(selected_uav_id_));
    mission_map_->setCenterCoordinate(uavCoordinate(selected_uav_id_).x(), uavCoordinate(selected_uav_id_).y());
    stack_widget_->setCurrentWidget(mission_page_);
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
        .arg(ui_->altitudeLineEdit->text())
        .arg(ui_->speedLineEdit->text()));
}

void UAVFleetGUI::setupUAVs() {
    ++uav_count_;
    populateUAVGrid();
    selectUAV(uav_count_);
    updateOutput(QString("Add UAVs created UAV %1 and added it to the fleet wall.").arg(uav_count_));
}

void UAVFleetGUI::removeUAV() {
    if (uav_count_ <= 1) {
        updateOutput("Cannot remove UAV. At least one UAV must remain on the fleet board.");
        return;
    }

    const int removed_id = selected_uav_id_;
    --uav_count_;
    if (selected_uav_id_ > uav_count_) {
        selected_uav_id_ = uav_count_;
    }

    populateUAVGrid();
    selectUAV(selected_uav_id_);
    updateOutput(QString("Removed UAV %1 from the fleet board.").arg(removed_id));
}

void UAVFleetGUI::startMission() {
    updateOutput(QString("Mission opened for UAV %1 using %2 mode.")
        .arg(selected_uav_id_)
        .arg(ui_->missionTypeCombo->currentText()));
    openMissionDialog();
}

void UAVFleetGUI::clearLogs() {
    ui_->logTextEdit->clear();
}

void UAVFleetGUI::saveLogs() {
    updateOutput("Logs saved from the operations panel.");
}

void UAVFleetGUI::updateTelemetry() {
    const QPointF coordinate = uavCoordinate(selected_uav_id_);
    ui_->positionValueLabel->setText(QString("%1, %2")
        .arg(QString::number(coordinate.x(), 'f', 4))
        .arg(QString::number(coordinate.y(), 'f', 4)));
    ui_->velocityValueLabel->setText(QString("%1 m/s").arg(ui_->speedLineEdit->text()));
    ui_->batteryValueLabel->setText(QString("%1%").arg(92 - ((selected_uav_id_ * 3) % 20)));
    ui_->gpsValueLabel->setText((selected_uav_id_ % 2 == 0) ? "3D Fix" : "RTK");
}

void UAVFleetGUI::updateOutput(const QString& text) {
    if (!ui_ || !ui_->logTextEdit) {
        return;
    }
    ui_->logTextEdit->append(text);
}
