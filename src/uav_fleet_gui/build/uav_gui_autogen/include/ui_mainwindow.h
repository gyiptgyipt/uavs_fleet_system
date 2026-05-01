/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_UAVFleetGUI
{
public:
    QWidget *centralwidget;
    QVBoxLayout *rootLayout;
    QFrame *topBar;
    QHBoxLayout *topBarLayout;
    QVBoxLayout *titleLayout;
    QLabel *titleLabel;
    QLabel *subtitleLabel;
    QLabel *fleetCountLabel;
    QPushButton *startSimButton;
    QSplitter *mainSplitter;
    QFrame *gridPanel;
    QVBoxLayout *gridPanelLayout;
    QHBoxLayout *fleetHeaderLayout;
    QLabel *fleetBoardLabel;
    QSpacerItem *fleetHeaderSpacer;
    QLabel *uavCountInputLabel;
    QLineEdit *uavCountLineEdit;
    QPushButton *saveUavCountButton;
    QScrollArea *fleetScrollArea;
    QWidget *fleetGridContainer;
    QGridLayout *fleetGridLayout;
    QSplitter *sideSplitter;
    QFrame *detailPanel;
    QVBoxLayout *detailLayout;
    QLabel *selectedUavTitleLabel;
    QLabel *selectedUavValueLabel;
    QGridLayout *metricsGridLayout;
    QFrame *positionFrame;
    QVBoxLayout *positionFrameLayout;
    QLabel *positionTitleLabel;
    QLabel *positionValueLabel;
    QFrame *velocityFrame;
    QVBoxLayout *velocityFrameLayout;
    QLabel *velocityTitleLabel;
    QLabel *velocityValueLabel;
    QFrame *batteryFrame;
    QVBoxLayout *batteryFrameLayout;
    QLabel *batteryTitleLabel;
    QLabel *batteryValueLabel;
    QFrame *gpsFrame;
    QVBoxLayout *gpsFrameLayout;
    QLabel *gpsTitleLabel;
    QLabel *gpsValueLabel;
    QLabel *missionTypeLabel;
    QComboBox *missionTypeCombo;
    QLabel *altitudeLabel;
    QLineEdit *altitudeLineEdit;
    QLabel *speedLabel;
    QLineEdit *speedLineEdit;
    QPushButton *armButton;
    QPushButton *setpointButton;
    QPushButton *openMissionButton;
    QFrame *logPanel;
    QVBoxLayout *logLayout;
    QLabel *operationsLogLabel;
    QTextEdit *logTextEdit;
    QHBoxLayout *logButtonsLayout;
    QPushButton *clearLogButton;
    QPushButton *saveLogButton;
    QSpacerItem *horizontalSpacer;

    void setupUi(QMainWindow *UAVFleetGUI)
    {
        if (UAVFleetGUI->objectName().isEmpty())
            UAVFleetGUI->setObjectName(QString::fromUtf8("UAVFleetGUI"));
        UAVFleetGUI->resize(1600, 910);
        centralwidget = new QWidget(UAVFleetGUI);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        rootLayout = new QVBoxLayout(centralwidget);
        rootLayout->setSpacing(16);
        rootLayout->setObjectName(QString::fromUtf8("rootLayout"));
        rootLayout->setContentsMargins(20, 18, 20, 18);
        topBar = new QFrame(centralwidget);
        topBar->setObjectName(QString::fromUtf8("topBar"));
        topBarLayout = new QHBoxLayout(topBar);
        topBarLayout->setSpacing(16);
        topBarLayout->setObjectName(QString::fromUtf8("topBarLayout"));
        topBarLayout->setContentsMargins(20, 12, 20, 12);
        titleLayout = new QVBoxLayout();
        titleLayout->setSpacing(2);
        titleLayout->setObjectName(QString::fromUtf8("titleLayout"));
        titleLabel = new QLabel(topBar);
        titleLabel->setObjectName(QString::fromUtf8("titleLabel"));

        titleLayout->addWidget(titleLabel);

        subtitleLabel = new QLabel(topBar);
        subtitleLabel->setObjectName(QString::fromUtf8("subtitleLabel"));

        titleLayout->addWidget(subtitleLabel);


        topBarLayout->addLayout(titleLayout);

        fleetCountLabel = new QLabel(topBar);
        fleetCountLabel->setObjectName(QString::fromUtf8("fleetCountLabel"));

        topBarLayout->addWidget(fleetCountLabel);

        startSimButton = new QPushButton(topBar);
        startSimButton->setObjectName(QString::fromUtf8("startSimButton"));

        topBarLayout->addWidget(startSimButton);


        rootLayout->addWidget(topBar);

        mainSplitter = new QSplitter(centralwidget);
        mainSplitter->setObjectName(QString::fromUtf8("mainSplitter"));
        mainSplitter->setOrientation(Qt::Horizontal);
        mainSplitter->setChildrenCollapsible(false);
        gridPanel = new QFrame(mainSplitter);
        gridPanel->setObjectName(QString::fromUtf8("gridPanel"));
        gridPanelLayout = new QVBoxLayout(gridPanel);
        gridPanelLayout->setSpacing(14);
        gridPanelLayout->setObjectName(QString::fromUtf8("gridPanelLayout"));
        gridPanelLayout->setContentsMargins(18, 18, 18, 18);
        fleetHeaderLayout = new QHBoxLayout();
        fleetHeaderLayout->setSpacing(12);
        fleetHeaderLayout->setObjectName(QString::fromUtf8("fleetHeaderLayout"));
        fleetBoardLabel = new QLabel(gridPanel);
        fleetBoardLabel->setObjectName(QString::fromUtf8("fleetBoardLabel"));

        fleetHeaderLayout->addWidget(fleetBoardLabel);

        fleetHeaderSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        fleetHeaderLayout->addItem(fleetHeaderSpacer);

        uavCountInputLabel = new QLabel(gridPanel);
        uavCountInputLabel->setObjectName(QString::fromUtf8("uavCountInputLabel"));

        fleetHeaderLayout->addWidget(uavCountInputLabel);

        uavCountLineEdit = new QLineEdit(gridPanel);
        uavCountLineEdit->setObjectName(QString::fromUtf8("uavCountLineEdit"));
        uavCountLineEdit->setMinimumSize(QSize(72, 0));
        uavCountLineEdit->setMaximumSize(QSize(96, 16777215));

        fleetHeaderLayout->addWidget(uavCountLineEdit);

        saveUavCountButton = new QPushButton(gridPanel);
        saveUavCountButton->setObjectName(QString::fromUtf8("saveUavCountButton"));

        fleetHeaderLayout->addWidget(saveUavCountButton);


        gridPanelLayout->addLayout(fleetHeaderLayout);

        fleetScrollArea = new QScrollArea(gridPanel);
        fleetScrollArea->setObjectName(QString::fromUtf8("fleetScrollArea"));
        fleetScrollArea->setWidgetResizable(true);
        fleetGridContainer = new QWidget();
        fleetGridContainer->setObjectName(QString::fromUtf8("fleetGridContainer"));
        fleetGridLayout = new QGridLayout(fleetGridContainer);
        fleetGridLayout->setObjectName(QString::fromUtf8("fleetGridLayout"));
        fleetGridLayout->setHorizontalSpacing(14);
        fleetGridLayout->setVerticalSpacing(14);
        fleetGridLayout->setContentsMargins(0, 0, 0, 0);
        fleetScrollArea->setWidget(fleetGridContainer);

        gridPanelLayout->addWidget(fleetScrollArea);

        mainSplitter->addWidget(gridPanel);
        sideSplitter = new QSplitter(mainSplitter);
        sideSplitter->setObjectName(QString::fromUtf8("sideSplitter"));
        sideSplitter->setOrientation(Qt::Vertical);
        sideSplitter->setChildrenCollapsible(false);
        detailPanel = new QFrame(sideSplitter);
        detailPanel->setObjectName(QString::fromUtf8("detailPanel"));
        detailLayout = new QVBoxLayout(detailPanel);
        detailLayout->setSpacing(14);
        detailLayout->setObjectName(QString::fromUtf8("detailLayout"));
        detailLayout->setContentsMargins(18, 18, 18, 18);
        selectedUavTitleLabel = new QLabel(detailPanel);
        selectedUavTitleLabel->setObjectName(QString::fromUtf8("selectedUavTitleLabel"));

        detailLayout->addWidget(selectedUavTitleLabel);

        selectedUavValueLabel = new QLabel(detailPanel);
        selectedUavValueLabel->setObjectName(QString::fromUtf8("selectedUavValueLabel"));

        detailLayout->addWidget(selectedUavValueLabel);

        metricsGridLayout = new QGridLayout();
        metricsGridLayout->setObjectName(QString::fromUtf8("metricsGridLayout"));
        metricsGridLayout->setHorizontalSpacing(12);
        metricsGridLayout->setVerticalSpacing(12);
        positionFrame = new QFrame(detailPanel);
        positionFrame->setObjectName(QString::fromUtf8("positionFrame"));
        positionFrameLayout = new QVBoxLayout(positionFrame);
        positionFrameLayout->setObjectName(QString::fromUtf8("positionFrameLayout"));
        positionFrameLayout->setContentsMargins(14, 12, 14, 12);
        positionTitleLabel = new QLabel(positionFrame);
        positionTitleLabel->setObjectName(QString::fromUtf8("positionTitleLabel"));

        positionFrameLayout->addWidget(positionTitleLabel);

        positionValueLabel = new QLabel(positionFrame);
        positionValueLabel->setObjectName(QString::fromUtf8("positionValueLabel"));

        positionFrameLayout->addWidget(positionValueLabel);


        metricsGridLayout->addWidget(positionFrame, 0, 0, 1, 1);

        velocityFrame = new QFrame(detailPanel);
        velocityFrame->setObjectName(QString::fromUtf8("velocityFrame"));
        velocityFrameLayout = new QVBoxLayout(velocityFrame);
        velocityFrameLayout->setObjectName(QString::fromUtf8("velocityFrameLayout"));
        velocityFrameLayout->setContentsMargins(14, 12, 14, 12);
        velocityTitleLabel = new QLabel(velocityFrame);
        velocityTitleLabel->setObjectName(QString::fromUtf8("velocityTitleLabel"));

        velocityFrameLayout->addWidget(velocityTitleLabel);

        velocityValueLabel = new QLabel(velocityFrame);
        velocityValueLabel->setObjectName(QString::fromUtf8("velocityValueLabel"));

        velocityFrameLayout->addWidget(velocityValueLabel);


        metricsGridLayout->addWidget(velocityFrame, 0, 1, 1, 1);

        batteryFrame = new QFrame(detailPanel);
        batteryFrame->setObjectName(QString::fromUtf8("batteryFrame"));
        batteryFrameLayout = new QVBoxLayout(batteryFrame);
        batteryFrameLayout->setObjectName(QString::fromUtf8("batteryFrameLayout"));
        batteryFrameLayout->setContentsMargins(14, 12, 14, 12);
        batteryTitleLabel = new QLabel(batteryFrame);
        batteryTitleLabel->setObjectName(QString::fromUtf8("batteryTitleLabel"));

        batteryFrameLayout->addWidget(batteryTitleLabel);

        batteryValueLabel = new QLabel(batteryFrame);
        batteryValueLabel->setObjectName(QString::fromUtf8("batteryValueLabel"));

        batteryFrameLayout->addWidget(batteryValueLabel);


        metricsGridLayout->addWidget(batteryFrame, 1, 0, 1, 1);

        gpsFrame = new QFrame(detailPanel);
        gpsFrame->setObjectName(QString::fromUtf8("gpsFrame"));
        gpsFrameLayout = new QVBoxLayout(gpsFrame);
        gpsFrameLayout->setObjectName(QString::fromUtf8("gpsFrameLayout"));
        gpsFrameLayout->setContentsMargins(14, 12, 14, 12);
        gpsTitleLabel = new QLabel(gpsFrame);
        gpsTitleLabel->setObjectName(QString::fromUtf8("gpsTitleLabel"));

        gpsFrameLayout->addWidget(gpsTitleLabel);

        gpsValueLabel = new QLabel(gpsFrame);
        gpsValueLabel->setObjectName(QString::fromUtf8("gpsValueLabel"));

        gpsFrameLayout->addWidget(gpsValueLabel);


        metricsGridLayout->addWidget(gpsFrame, 1, 1, 1, 1);


        detailLayout->addLayout(metricsGridLayout);

        missionTypeLabel = new QLabel(detailPanel);
        missionTypeLabel->setObjectName(QString::fromUtf8("missionTypeLabel"));

        detailLayout->addWidget(missionTypeLabel);

        missionTypeCombo = new QComboBox(detailPanel);
        missionTypeCombo->setObjectName(QString::fromUtf8("missionTypeCombo"));

        detailLayout->addWidget(missionTypeCombo);

        altitudeLabel = new QLabel(detailPanel);
        altitudeLabel->setObjectName(QString::fromUtf8("altitudeLabel"));

        detailLayout->addWidget(altitudeLabel);

        altitudeLineEdit = new QLineEdit(detailPanel);
        altitudeLineEdit->setObjectName(QString::fromUtf8("altitudeLineEdit"));

        detailLayout->addWidget(altitudeLineEdit);

        speedLabel = new QLabel(detailPanel);
        speedLabel->setObjectName(QString::fromUtf8("speedLabel"));

        detailLayout->addWidget(speedLabel);

        speedLineEdit = new QLineEdit(detailPanel);
        speedLineEdit->setObjectName(QString::fromUtf8("speedLineEdit"));

        detailLayout->addWidget(speedLineEdit);

        armButton = new QPushButton(detailPanel);
        armButton->setObjectName(QString::fromUtf8("armButton"));

        detailLayout->addWidget(armButton);

        setpointButton = new QPushButton(detailPanel);
        setpointButton->setObjectName(QString::fromUtf8("setpointButton"));

        detailLayout->addWidget(setpointButton);

        openMissionButton = new QPushButton(detailPanel);
        openMissionButton->setObjectName(QString::fromUtf8("openMissionButton"));

        detailLayout->addWidget(openMissionButton);

        sideSplitter->addWidget(detailPanel);
        logPanel = new QFrame(sideSplitter);
        logPanel->setObjectName(QString::fromUtf8("logPanel"));
        logLayout = new QVBoxLayout(logPanel);
        logLayout->setSpacing(12);
        logLayout->setObjectName(QString::fromUtf8("logLayout"));
        logLayout->setContentsMargins(18, 18, 18, 18);
        operationsLogLabel = new QLabel(logPanel);
        operationsLogLabel->setObjectName(QString::fromUtf8("operationsLogLabel"));

        logLayout->addWidget(operationsLogLabel);

        logTextEdit = new QTextEdit(logPanel);
        logTextEdit->setObjectName(QString::fromUtf8("logTextEdit"));

        logLayout->addWidget(logTextEdit);

        logButtonsLayout = new QHBoxLayout();
        logButtonsLayout->setObjectName(QString::fromUtf8("logButtonsLayout"));
        clearLogButton = new QPushButton(logPanel);
        clearLogButton->setObjectName(QString::fromUtf8("clearLogButton"));

        logButtonsLayout->addWidget(clearLogButton);

        saveLogButton = new QPushButton(logPanel);
        saveLogButton->setObjectName(QString::fromUtf8("saveLogButton"));

        logButtonsLayout->addWidget(saveLogButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        logButtonsLayout->addItem(horizontalSpacer);


        logLayout->addLayout(logButtonsLayout);

        sideSplitter->addWidget(logPanel);
        mainSplitter->addWidget(sideSplitter);

        rootLayout->addWidget(mainSplitter);

        UAVFleetGUI->setCentralWidget(centralwidget);

        retranslateUi(UAVFleetGUI);

        QMetaObject::connectSlotsByName(UAVFleetGUI);
    } // setupUi

    void retranslateUi(QMainWindow *UAVFleetGUI)
    {
        UAVFleetGUI->setWindowTitle(QCoreApplication::translate("UAVFleetGUI", "UAV Fleet", nullptr));
        titleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "UAVs Web Management System", nullptr));
        subtitleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Monitor and control your drone", nullptr));
        fleetCountLabel->setText(QCoreApplication::translate("UAVFleetGUI", "10 UAVs online", nullptr));
        startSimButton->setText(QCoreApplication::translate("UAVFleetGUI", "Start PX4 Simulation", nullptr));
        fleetBoardLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Fleet board", nullptr));
        uavCountInputLabel->setText(QCoreApplication::translate("UAVFleetGUI", "UAV count", nullptr));
        uavCountLineEdit->setText(QCoreApplication::translate("UAVFleetGUI", "10", nullptr));
        saveUavCountButton->setText(QCoreApplication::translate("UAVFleetGUI", "Save", nullptr));
        selectedUavTitleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Selected UAV", nullptr));
        selectedUavValueLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Drone 1", nullptr));
        positionTitleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Position", nullptr));
        positionValueLabel->setText(QCoreApplication::translate("UAVFleetGUI", "N/A", nullptr));
        velocityTitleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Velocity", nullptr));
        velocityValueLabel->setText(QCoreApplication::translate("UAVFleetGUI", "N/A", nullptr));
        batteryTitleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Battery", nullptr));
        batteryValueLabel->setText(QCoreApplication::translate("UAVFleetGUI", "N/A", nullptr));
        gpsTitleLabel->setText(QCoreApplication::translate("UAVFleetGUI", "GPS", nullptr));
        gpsValueLabel->setText(QCoreApplication::translate("UAVFleetGUI", "N/A", nullptr));
        missionTypeLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Mission Type", nullptr));
        altitudeLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Altitude (m)", nullptr));
        altitudeLineEdit->setText(QCoreApplication::translate("UAVFleetGUI", "20", nullptr));
        speedLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Cruise Speed (m/s)", nullptr));
        speedLineEdit->setText(QCoreApplication::translate("UAVFleetGUI", "2.5", nullptr));
        armButton->setText(QCoreApplication::translate("UAVFleetGUI", "Arm Selected UAV", nullptr));
        setpointButton->setText(QCoreApplication::translate("UAVFleetGUI", "Send Setpoints", nullptr));
        openMissionButton->setText(QCoreApplication::translate("UAVFleetGUI", "Open Mission", nullptr));
        operationsLogLabel->setText(QCoreApplication::translate("UAVFleetGUI", "Operations Log", nullptr));
        clearLogButton->setText(QCoreApplication::translate("UAVFleetGUI", "Clear", nullptr));
        saveLogButton->setText(QCoreApplication::translate("UAVFleetGUI", "Save", nullptr));
    } // retranslateUi

};

namespace Ui {
    class UAVFleetGUI: public Ui_UAVFleetGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
