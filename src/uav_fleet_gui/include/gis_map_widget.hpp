#ifndef UAV_FLEET_GUI_GIS_MAP_WIDGET_HPP
#define UAV_FLEET_GUI_GIS_MAP_WIDGET_HPP

#include <QHash>
#include <QNetworkAccessManager>
#include <QPixmap>
#include <QPoint>
#include <QSet>
#include <QWidget>

class GISMapWidget : public QWidget {
    Q_OBJECT

public:
    explicit GISMapWidget(QWidget* parent = nullptr);

    void setCenterCoordinate(double latitude, double longitude);
    void setZoomLevel(int zoom);
    bool hasHeightForWidth() const override;
    int heightForWidth(int width) const override;
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    QString tileKey(int zoom, int x, int y) const;
    void requestTile(int zoom, int x, int y);
    double worldXFromLongitude(double longitude, int zoom) const;
    double worldYFromLatitude(double latitude, int zoom) const;
    double longitudeFromWorldX(double world_x, int zoom) const;
    double latitudeFromWorldY(double world_y, int zoom) const;

    QNetworkAccessManager network_manager_;
    QHash<QString, QPixmap> tile_cache_;
    QSet<QString> inflight_requests_;

    double center_latitude_;
    double center_longitude_;
    int zoom_level_;

    bool dragging_;
    QPoint last_mouse_pos_;
};

#endif // UAV_FLEET_GUI_GIS_MAP_WIDGET_HPP
