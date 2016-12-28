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

namespace cslibs_gdal {
class Map;
class View;

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

private slots:
    void postRendering();

private:
    QGraphicsView  *view_;
    QGraphicsScene *scene_;
    std::mutex      scene_mutex_;
    QRectF          scene_rect_;

    std::map<QString, QGraphicsPathItem*>   paths_;
    QPen                                    default_pen_;

    Map                                    *map_;

    std::thread                             worker_thread_;
    bool                                    stop_;

    void run();
    void doRepaint();
    void doRepaint(const QString &name);
    void doUpdate(const QString &name);

    mutable std::mutex                      render_queue_mutex_;
    std::queue<std::function<void()>>       render_queue_;
    std::condition_variable                 render_condition_;

};
}
#endif // RENDERER_H
