#include "gis_map_widget.hpp"

#include <QMouseEvent>
#include <QNetworkReply>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>
#include <QtMath>

GISMapWidget::GISMapWidget(QWidget* parent)
    : QWidget(parent),
      center_latitude_(22.48),
      center_longitude_(94.94),
      zoom_level_(8),
      dragging_(false),
      drag_moved_(false) {
    setMinimumHeight(120);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    setMouseTracking(true);
}

bool GISMapWidget::hasHeightForWidth() const {
    return true;
}

int GISMapWidget::heightForWidth(int width) const {
    const int preferred = (width * 150) / 320;
    return qBound(120, preferred, 210);
}

QSize GISMapWidget::sizeHint() const {
    return QSize(320, 150);
}

QSize GISMapWidget::minimumSizeHint() const {
    return QSize(220, 120);
}

void GISMapWidget::setCenterCoordinate(double latitude, double longitude) {
    center_latitude_ = qBound(-85.0, latitude, 85.0);
    center_longitude_ = longitude;
    update();
}

void GISMapWidget::setZoomLevel(int zoom) {
    zoom_level_ = qBound(2, zoom, 18);
    update();
}

void GISMapWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QPainterPath clip_path;
    clip_path.addRoundedRect(rect(), 16, 16);
    painter.setClipPath(clip_path);
    painter.fillRect(rect(), QColor("#171520"));

    const int tile_size = 256;
    const double world_x = worldXFromLongitude(center_longitude_, zoom_level_);
    const double world_y = worldYFromLatitude(center_latitude_, zoom_level_);
    const double origin_x = world_x - (width() / 2.0);
    const double origin_y = world_y - (height() / 2.0);

    const int first_tile_x = static_cast<int>(std::floor(origin_x / tile_size));
    const int first_tile_y = static_cast<int>(std::floor(origin_y / tile_size));
    const int last_tile_x = static_cast<int>(std::floor((origin_x + width()) / tile_size));
    const int last_tile_y = static_cast<int>(std::floor((origin_y + height()) / tile_size));
    const int world_tile_count = 1 << zoom_level_;

    for (int tile_y = first_tile_y; tile_y <= last_tile_y; ++tile_y) {
        for (int tile_x = first_tile_x; tile_x <= last_tile_x; ++tile_x) {
            if (tile_y < 0 || tile_y >= world_tile_count) {
                continue;
            }

            const int wrapped_x = (tile_x % world_tile_count + world_tile_count) % world_tile_count;
            const QString key = tileKey(zoom_level_, wrapped_x, tile_y);
            const QRect target_rect(
                static_cast<int>(tile_x * tile_size - origin_x),
                static_cast<int>(tile_y * tile_size - origin_y),
                tile_size,
                tile_size);

            if (tile_cache_.contains(key)) {
                painter.drawPixmap(target_rect, tile_cache_.value(key));
            } else {
                painter.fillRect(target_rect, QColor("#1f1b2b"));
                painter.setPen(QColor(255, 255, 255, 30));
                painter.drawRect(target_rect.adjusted(0, 0, -1, -1));
                painter.drawText(target_rect, Qt::AlignCenter, "Loading map...");
                requestTile(zoom_level_, wrapped_x, tile_y);
            }
        }
    }

    painter.setPen(QColor(248, 248, 242, 220));
    painter.setFont(QFont("Noto Sans", 10, QFont::Bold));
    painter.drawText(QRect(12, 12, 240, 20),
        QString("ArcGIS Imagery  |  Zoom %1").arg(zoom_level_));
    painter.drawText(QRect(12, height() - 26, 280, 18),
        QString("Lat %1  Lon %2")
            .arg(center_latitude_, 0, 'f', 4)
            .arg(center_longitude_, 0, 'f', 4));
}

void GISMapWidget::wheelEvent(QWheelEvent* event) {
    const int delta = event->angleDelta().y();
    if (delta > 0) {
        setZoomLevel(zoom_level_ + 1);
    } else if (delta < 0) {
        setZoomLevel(zoom_level_ - 1);
    }
    event->accept();
}

void GISMapWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        drag_moved_ = false;
        last_mouse_pos_ = event->pos();
        mouse_press_pos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void GISMapWidget::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) {
        QWidget::mouseMoveEvent(event);
        return;
    }

    const QPoint delta = event->pos() - last_mouse_pos_;
    last_mouse_pos_ = event->pos();
    if ((event->pos() - mouse_press_pos_).manhattanLength() > 4) {
        drag_moved_ = true;
    }

    const double world_x = worldXFromLongitude(center_longitude_, zoom_level_) - delta.x();
    const double world_y = worldYFromLatitude(center_latitude_, zoom_level_) - delta.y();
    center_longitude_ = longitudeFromWorldX(world_x, zoom_level_);
    center_latitude_ = latitudeFromWorldY(world_y, zoom_level_);
    update();
    event->accept();
}

void GISMapWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        if (!drag_moved_) {
            const double center_world_x = worldXFromLongitude(center_longitude_, zoom_level_);
            const double center_world_y = worldYFromLatitude(center_latitude_, zoom_level_);
            const double clicked_world_x = center_world_x + event->pos().x() - (width() / 2.0);
            const double clicked_world_y = center_world_y + event->pos().y() - (height() / 2.0);
            const double clicked_latitude = latitudeFromWorldY(clicked_world_y, zoom_level_);
            const double clicked_longitude = longitudeFromWorldX(clicked_world_x, zoom_level_);
            emit mapClicked(clicked_latitude, clicked_longitude);
        }
        dragging_ = false;
        drag_moved_ = false;
        unsetCursor();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

QString GISMapWidget::tileKey(int zoom, int x, int y) const {
    return QString("%1_%2_%3").arg(zoom).arg(x).arg(y);
}

void GISMapWidget::requestTile(int zoom, int x, int y) {
    const QString key = tileKey(zoom, x, y);
    if (tile_cache_.contains(key) || inflight_requests_.contains(key)) {
        return;
    }

    inflight_requests_.insert(key);
    const QUrl url(QString(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%1/%2/%3")
        .arg(zoom)
        .arg(y)
        .arg(x));

    QNetworkRequest request(url);
    request.setRawHeader("User-Agent", "uav_fleet_gui/0.0.1");

    QNetworkReply* reply = network_manager_.get(request);
    connect(reply, &QNetworkReply::finished, this, [this, reply, key]() {
        reply->deleteLater();
        inflight_requests_.remove(key);

        if (reply->error() != QNetworkReply::NoError) {
            update();
            return;
        }

        QPixmap tile;
        if (tile.loadFromData(reply->readAll())) {
            tile_cache_.insert(key, tile);
        }
        update();
    });
}

double GISMapWidget::worldXFromLongitude(double longitude, int zoom) const {
    const double tiles = 1 << zoom;
    return ((longitude + 180.0) / 360.0) * tiles * 256.0;
}

double GISMapWidget::worldYFromLatitude(double latitude, int zoom) const {
    const double tiles = 1 << zoom;
    const double lat_rad = qDegreesToRadians(qBound(-85.0, latitude, 85.0));
    const double mercator = std::log(std::tan(M_PI / 4.0 + lat_rad / 2.0));
    return (1.0 - mercator / M_PI) / 2.0 * tiles * 256.0;
}

double GISMapWidget::longitudeFromWorldX(double world_x, int zoom) const {
    const double tiles = 1 << zoom;
    return (world_x / (tiles * 256.0)) * 360.0 - 180.0;
}

double GISMapWidget::latitudeFromWorldY(double world_y, int zoom) const {
    const double tiles = 1 << zoom;
    const double mercator = M_PI - (2.0 * M_PI * world_y) / (tiles * 256.0);
    return qRadiansToDegrees(std::atan(std::sinh(mercator)));
}
