#include "renderer.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPainterPath>
#include <QGraphicsPathItem>

#include "view.h"
#include "map.h"
#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"
#include "models/corner_layer_model.h"
#include "util/bezier_color.hpp"
#include "util/hsv.hpp"

using namespace cslibs_vectormaps;

Renderer::Renderer() :
    default_point_alpha_(0.6),
    stop_(false)
{
}

Renderer::~Renderer()
{
    stop_ = true;
    render_condition_.notify_one();
    try{
        worker_thread_.join();
    } catch(const std::exception &e) {
        std::cerr << e.what() << "\n";
    }

    delete scene_;
}

void Renderer::setup(Map *map, QGraphicsView *grahpics_view)
{
    map_ = map;
    view_ = grahpics_view;
    scene_ = new QGraphicsScene();
    view_->setOptimizationFlags(QGraphicsView::DontSavePainterState);

    view_->setScene(scene_);

    connect(this, SIGNAL(finished()), this, SLOT(postRendering()), Qt::QueuedConnection);
    connect(this, SIGNAL(clear()), this, SLOT(clearScene()), Qt::QueuedConnection);
    connect(this, SIGNAL(add(QGraphicsItemGroup*)), this, SLOT(addGroup(QGraphicsItemGroup*)), Qt::QueuedConnection);

    worker_thread_ = std::thread([this](){run();});
}

void Renderer::setDefaultPen(const QPen &pen)
{
    default_pen_ = pen;
}

void Renderer::run()
{
    while(!stop_) {
        std::unique_lock<std::mutex> l(render_queue_mutex_);
        while(render_queue_.empty() && !stop_)
            render_condition_.wait(l);

        if(!render_queue_.empty()) {
            render_queue_.front()();
            render_queue_.pop();
        }

        if(stop_) {
            return;
        }
    }
}

void Renderer::repaint()
{
    std::unique_lock<std::mutex> l(render_queue_mutex_);
    render_queue_.push([this](){doRepaint(); finished();});
    l.unlock();
    render_condition_.notify_one();
}

void Renderer::repaint(const QString &layer_name)
{
    std::unique_lock<std::mutex> l(render_queue_mutex_);
    render_queue_.push([this, layer_name](){doRepaint(layer_name); finished();});
    l.unlock();
    render_condition_.notify_one();
}

void Renderer::update(const QString &layer_name)
{
    std::unique_lock<std::mutex> l(render_queue_mutex_);
    render_queue_.push([this, layer_name](){doUpdate(layer_name); finished();});
    l.unlock();
    render_condition_.notify_one();
}

void Renderer::postRendering()
{
    view_->update();
}

void Renderer::clearScene()
{
    scene_->clear();
}

void Renderer::addGroup(QGraphicsItemGroup *g)
{
    scene_->addItem(g);
}

void Renderer::doRepaint()
{
    clear();
    groups_.clear();

    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    for(auto &l : layers) {
        doRepaint(l->getName<QString>());
    }
}

void Renderer::doRepaint(const QString &layer_name)
{
    LayerModel::ConstPtr l = map_->getLayer(layer_name);
    QGraphicsItemGroup *g = nullptr;

    /// TODO : REFACTORING TO DOUBLE CALL PATTERN

    VectorLayerModel::ConstPtr lv = LayerModel::as<VectorLayerModel const>(l);
    if(lv) {
        QGraphicsItemGroup *g = new QGraphicsItemGroup;
        render(*lv, *g);
        groups_[layer_name] = g;
        g->setVisible(l->getVisibility());

        add(g);
    }
    CornerLayerModel::ConstPtr cl = LayerModel::as<CornerLayerModel const>(l);
    if(cl) {
        QGraphicsItemGroup *g = new QGraphicsItemGroup;
        render(*cl, *g);
        groups_[layer_name] = g;
        g->setVisible(l->getVisibility());

        add(g);
   }
    PointLayerModel::ConstPtr lp = LayerModel::as<PointLayerModel const>(l);
    if(lp && !cl) {
        QGraphicsItemGroup *g = new QGraphicsItemGroup;
        render(*lp, *g);
        groups_[layer_name] = g;
        g->setVisible(l->getVisibility());

        add(g);
    }
}

void Renderer::doUpdate(const QString &layer_name)
{
    LayerModel::ConstPtr l = map_->getLayer(layer_name);
    QGraphicsItemGroup *g = groups_[layer_name];

    VectorLayerModel::ConstPtr lv = LayerModel::as<VectorLayerModel const>(l);
    if(lv) {
        update(*lv, g);
    }
    CornerLayerModel::ConstPtr cl = LayerModel::as<CornerLayerModel const>(l);
    if(cl) {
        update(*cl, g);
    }
    PointLayerModel::ConstPtr  lp = LayerModel::as<PointLayerModel const>(l);
    if(lp && !cl) {
        update(*lp, g);
    }
    g->setVisible(l->getVisibility());
}

void Renderer::render(const CornerLayerModel &model,
                      QGraphicsItemGroup &group)
{
    QColor color = model.getColor();
    QPen    p(default_pen_);
    p.setColor(Qt::black);

    std::vector<QPointF> points;
    std::vector<double>  cornerness;
    model.getPoints(points);
    model.getCornerness(cornerness);

    for(std::size_t i = 0 ; i < points.size(); ++i) {
        const QPointF &point = points[i];
        const double  &corner = cornerness[i];
        color.setAlphaF(corner * default_point_alpha_);
        QBrush  b(color);
        QGraphicsEllipseItem *item = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        item->setBrush(b);
        item->setPen(p);
        group.addToGroup(item);
    }

}

void Renderer::render(const PointLayerModel &model,
                      QGraphicsItemGroup &group)
{
    QPen    p(default_pen_);
    p.setColor(Qt::black);
    QColor  c(model.getColor());
    c.setAlphaF(default_point_alpha_);
    QBrush  b(c);

    QGraphicsItemGroup *g = new QGraphicsItemGroup;
    std::vector<QPointF> points;
    model.getPoints(points);
    for(const QPointF &point : points) {
        QGraphicsEllipseItem *i = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        i->setPen(p);
        i->setBrush(b);
        group.addToGroup(i);
    }
}

void Renderer::render(const VectorLayerModel &model,
                      QGraphicsItemGroup &group)
{
    QPen    p(default_pen_);
    p.setColor(model.getColor());
    std::vector<QLineF> lines;
    model.getVectors(lines);
    for(auto l : lines) {
        QGraphicsLineItem *i = new QGraphicsLineItem(QLineF(l.p1(), l.p2()));
        i->setPen(p);
        group.addToGroup(i);
    }
}

void Renderer::update(const CornerLayerModel &model,
                      QGraphicsItemGroup *group)
{
    QColor color = model.getColor();
    QPen    p(default_pen_);
    p.setColor(Qt::black);

    std::vector<double>  cornerness;
    model.getCornerness(cornerness);

    QList<QGraphicsItem*> children = group->childItems();
    assert(cornerness.size() == children.size());
    for(std::size_t i = 0 ; i < children.size(); ++i) {
        QGraphicsEllipseItem *item = static_cast<QGraphicsEllipseItem*>(children[i]);
        const double  &corner = cornerness[i];
        color.setAlphaF(corner * default_point_alpha_);
        QBrush  b(color);
        item->setBrush(b);
        item->setPen(p);
    }
}

void Renderer::update(const PointLayerModel &model,
                      QGraphicsItemGroup *group)
{
    QColor c(model.getColor());
    c.setAlphaF(default_point_alpha_);
    QBrush b(c);
    QList<QGraphicsItem*> children = group->childItems();
    for(auto *child : children) {
        static_cast<QGraphicsEllipseItem*>(child)->setBrush(b);
    }
}

void Renderer::update(const VectorLayerModel &model,
                      QGraphicsItemGroup *group)
{
    QColor c(model.getColor());
    QPen    p(default_pen_);
    p.setColor(c);
    QList<QGraphicsItem*> children = group->childItems();
    for(auto *child : children) {
        static_cast<QGraphicsLineItem*>(child)->setPen(p);
    }
}


