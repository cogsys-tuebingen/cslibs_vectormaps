#ifndef RENDERER_H
#define RENDERER_H

#include <QObject>
#include <QRectF>
#include <QPen>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsPathItem;

namespace cslibs_gdal {
class Map;
class View;

class Renderer : QObject
{
    Q_OBJECT

public:
    Renderer();

    void setup(View *view,
               Map *map,
               QGraphicsView *grahpics_view);

    void setDefaultPen(const QPen &pen);

public slots:
    void repaint();
    void repaint(const QString &layer_name);
    void update(const QString &layer_name);

private:
    QGraphicsView  *view_;
    QGraphicsScene *scene_;
    QRectF          scene_rect_;

    std::map<QString, QGraphicsPathItem*>   paths_;
    QPen                                    default_pen_;

    Map                                    *map_;

};
}
#endif // RENDERER_H
