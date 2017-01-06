#ifndef RENDERER_H
#define RENDERER_H

#include <QObject>
#include <QRectF>
#include <QPen>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <atomic>
#include <functional>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsPathItem;
class QGraphicsItemGroup;

namespace cslibs_gdal {
class Map;
class View;

/// forward declarations for rendering
class VectorLayerModel;
class CornerLayerModel;
class PointLayerModel;


class Renderer : QObject
{
    Q_OBJECT

public:
    Renderer();
    ~Renderer();

    void setup(Map *map,
               QGraphicsView *grahpics_view);

    void setDefaultPen(const QPen &pen);

    void repaint();
    void repaint(const QString &layer_name);
    void update(const QString &layer_name);

signals:
    void finished();
    void clear();
    void add(QGraphicsItemGroup *g);

private slots:
    void postRendering();
    void clearScene();
    void addGroup(QGraphicsItemGroup *g);

private:
    QGraphicsView  *view_;
    QGraphicsScene *scene_;
    std::mutex      scene_mutex_;
    QRectF          scene_rect_;

    std::map<QString, QGraphicsItemGroup*>   groups_;
    QPen                                     default_pen_;
    double                                   default_point_alpha_;

    Map                                     *map_;

    std::thread                              worker_thread_;
    bool                                     stop_;

    mutable std::mutex                      render_queue_mutex_;
    std::queue<std::function<void()>>       render_queue_;
    std::condition_variable                 render_condition_;

    void run();
    void doRepaint();
    void doRepaint(const QString &name);
    void doUpdate(const QString &name);

    void render(const CornerLayerModel &model,
                QGraphicsItemGroup &group);
    void render(const PointLayerModel  &model,
                QGraphicsItemGroup &group);
    void render(const VectorLayerModel &model,
                QGraphicsItemGroup &group);

    void update(const VectorLayerModel &model,
                QGraphicsItemGroup *group);
    void update(const CornerLayerModel &model,
                QGraphicsItemGroup *group);
    void update(const PointLayerModel  &model,
                QGraphicsItemGroup *group);



};
}
#endif // RENDERER_H
