#ifndef RENDERER_H
#define RENDERER_H

#include <QObject>
#include <QString>
#include <QRectF>
#include <QPen>

#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>
#include <functional>

class QGraphicsView;
class QGraphicsScene;
class QGraphicsItemGroup;

namespace cslibs_vectormaps {
class Map;

class Renderer : QObject
{
    Q_OBJECT

public:
    Renderer();
    ~Renderer();

    void setup(Map *map, QGraphicsView *graphics_view);

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
};
}
#endif // RENDERER_H
