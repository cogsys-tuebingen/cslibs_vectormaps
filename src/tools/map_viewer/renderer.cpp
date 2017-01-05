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

using namespace cslibs_gdal;

Renderer::Renderer() :
    default_point_alpha_(0.8),
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
        std::cerr << e.what() << std::endl;
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

void Renderer::doRepaint()
{
    scene_->clear();

    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    /// create new path items
    for(auto l = layers.begin() ; l < layers.end() ; ++l) {
        doRepaint((*l)->getName<QString>());
    }
}

void Renderer::doRepaint(const QString &layer_name)
{
    LayerModel::ConstPtr l = map_->getLayer(layer_name);
    QGraphicsItemGroup *g = nullptr;

    VectorLayerModel::ConstPtr lv = LayerModel::as<VectorLayerModel const>(l);
    if(lv) {
        g = render(*lv);
    }
    CornerLayerModel::ConstPtr cl = LayerModel::as<CornerLayerModel const>(l);
    if(cl) {
        g = render(*cl);
    }
    PointLayerModel::ConstPtr lp = LayerModel::as<PointLayerModel const>(l);
    if(lp && !cl) {
        g = render(*lp);
    }
    if(g) {
        scene_->addItem(g);
        groups_[layer_name] = g;
        g->setVisible(l->getVisibility());
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

QGraphicsItemGroup* Renderer::render(const CornerLayerModel &model)
{
    QColor color = model.getColor();
    color.setAlphaF(default_point_alpha_);
    QColor s1 = hsv::shiftHue(color, -135.); /// support color 1
    QColor s2 = hsv::shiftHue(color,  135.); /// support color 2

    bezier  bez({s1, color, s2});
    QPen    p(default_pen_);
    p.setColor(Qt::black);

    QGraphicsItemGroup *g = new QGraphicsItemGroup;
    std::vector<QPointF> points;
    std::vector<double>  cornerness;
    model.getPoints(points);
    model.getCornerness(cornerness);

    for(std::size_t i = 0 ; i < points.size(); ++i) {
        const QPointF &point = points[i];
        const double  &corner = cornerness[i];
        QBrush  b(bez.get(corner));
        QGraphicsEllipseItem *item = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        item->setPen(p);
        item->setBrush(b);
        g->addToGroup(item);
    }

    return g;
}

QGraphicsItemGroup* Renderer::render(const PointLayerModel &model)
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
        g->addToGroup(i);
    }
    return g;
}

QGraphicsItemGroup* Renderer::render(const VectorLayerModel &model)
{
    QPen    p(default_pen_);
    p.setColor(model.getColor());
    QGraphicsItemGroup *g = new QGraphicsItemGroup;
    std::vector<QLineF> lines;
    model.getVectors(lines);
    for(auto l : lines) {
        QGraphicsLineItem *i = new QGraphicsLineItem(QLineF(l.p1(), l.p2()));
        i->setPen(p);
        g->addToGroup(i);
    }
    return g;
}

void Renderer::update(const CornerLayerModel &model,
                      QGraphicsItemGroup *group)
{
    QColor color = model.getColor();
    color.setAlphaF(default_point_alpha_);
    QColor s1 = hsv::shiftHue(color, -135.); /// support color 1
    QColor s2 = hsv::shiftHue(color,  135.); /// support color 2

    bezier  bez({s1, color, s2});
    QPen    p(default_pen_);
    p.setColor(Qt::black);

    std::vector<double>  cornerness;
    model.getCornerness(cornerness);

    QList<QGraphicsItem*> children = group->childItems();
    assert(cornerness.size() == children.size());
    for(std::size_t i = 0 ; i < children.size(); ++i) {
        QGraphicsEllipseItem *item = static_cast<QGraphicsEllipseItem*>(children[i]);
        const double  &corner = cornerness[i];
        QBrush  b(bez.get(corner));
        item->setBrush(b);
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
    QPen p(c);
    QList<QGraphicsItem*> children = group->childItems();
    for(auto *child : children) {
        static_cast<QGraphicsLineItem*>(child)->setPen(p);
    }
}


