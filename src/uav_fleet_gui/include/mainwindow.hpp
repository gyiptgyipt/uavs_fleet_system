#ifndef UAV_FLEET_GUI_MAINWINDOW_HPP
#define UAV_FLEET_GUI_MAINWINDOW_HPP

#include <QGridLayout>
#include <QHash>
#include <QMainWindow>
#include <QNetworkAccessManager>
#include <QPixmap>
#include <QPushButton>
#include <QThread>
#include <QTimer>

class GISMapWidget;
class UAVControlNode;

namespace Ui {
class UAVFleetGUI;
}

class SimulationThread : public QThread {
    Q_OBJECT
public:
    explicit SimulationThread(const QString& command);
    void run() override;

signals:
    void outputReceived(const QString& output);

private:
    QString command_;
};

class UAVFleetGUI : public QMainWindow {
    Q_OBJECT

public:
    explicit UAVFleetGUI(QWidget *parent = nullptr);
    ~UAVFleetGUI();

private slots:
    void startSimulation();
    void armUAVs();
    void sendSetpoints();
    void setupUAVs();
    void removeUAV();
    void startMission();
    void clearLogs();
    void saveLogs();
    void updateTelemetry();
    void updateOutput(const QString& text);

private:
    void setupUI();
    void populateUAVGrid();
    QWidget* createUAVCard(int uav_id);
    QPixmap createMapPreview(int uav_id) const;
    QPointF uavCoordinate(int uav_id) const;
    void requestMapPreview(QPushButton* map_button, int uav_id);
    void selectUAV(int uav_id);

    Ui::UAVFleetGUI* ui_;
    GISMapWidget* interactive_map_;
    UAVControlNode* node_;
    SimulationThread* sim_thread_;

    QWidget* uav_grid_container_;
    QGridLayout* uav_grid_layout_;

    QTimer* telemetry_timer_;
    QNetworkAccessManager* map_network_manager_;
    QHash<int, QPixmap> map_preview_cache_;

    int selected_uav_id_;
    int uav_count_;
};

#endif // UAV_FLEET_GUI_MAINWINDOW_HPP
