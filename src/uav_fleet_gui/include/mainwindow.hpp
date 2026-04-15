#ifndef UAV_FLEET_GUI_MAINWINDOW_HPP
#define UAV_FLEET_GUI_MAINWINDOW_HPP

#include <QMainWindow>
#include <QSplitter>
#include <QTabWidget>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QListWidget>
#include <QTableWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QTextEdit>
#include <QTimer>
#include <QThread>

class UAVControlNode;

class SimulationThread : public QThread {
    Q_OBJECT
public:
    SimulationThread(const QString& command);
    void run() override;

signals:
    void outputReceived(const QString& output);

private:
    QString command_;
};

class UAVFleetGUI : public QMainWindow {
    Q_OBJECT

public:
    UAVFleetGUI(QWidget *parent = nullptr);
    ~UAVFleetGUI();

private slots:
    void startSimulation();
    void armUAVs();
    void sendSetpoints();
    void addWaypoint();
    void deleteWaypoint(int row);
    void startMission();
    void clearLogs();
    void saveLogs();
    void updateTelemetry();
    void updateOutput(const QString& text);

private:
    void setupUI();
    QWidget* createLeftPanel();
    QTabWidget* createRightPanel();
    QWidget* createDashboardTab();
    QWidget* createMissionTab();
    QWidget* createTelemetryTab();
    QWidget* createLogsTab();

    UAVControlNode* node_;
    SimulationThread* sim_thread_;

    // UI Elements
    QListWidget* uav_list_;
    QTableWidget* status_table_;
    QPushButton* start_sim_btn_;
    QPushButton* arm_btn_;
    QPushButton* setpoint_btn_;

    QTableWidget* waypoint_table_;
    QComboBox* mission_type_;
    QLineEdit* altitude_input_;
    QLineEdit* speed_input_;

    QLabel* selected_uav_telemetry_;
    QLabel* position_label_;
    QLabel* velocity_label_;
    QLabel* battery_label_;
    QLabel* gps_label_;

    QTextEdit* output_text_;

    QTimer* telemetry_timer_;
};

#endif // UAV_FLEET_GUI_MAINWINDOW_HPP