#include "renderer.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QPainterPath>
#include <QGraphicsPathItem>

#include "view.h"
#include "map.h"
#include "models/vector_layer_model.h"
#include "models/point_layer_model.h"

using namespace cslibs_gdal;

Renderer::Renderer() :
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

    QPainterPath painter;
    painter.setFillRule(Qt::WindingFill);
    QPen p = default_pen_;
    QColor color = l->getColor();
    p.setColor(color);
    color.setAlpha(127);
    QBrush b(color);

    VectorLayerModel::ConstPtr lv = LayerModel::as<VectorLayerModel const>(l);
    if(lv) {
        std::vector<QLineF> lines;
        lv->getVectors(lines);
        for(const QLineF &l : lines) {
            painter.moveTo(l.p1());
            painter.lineTo(l.p2());
        }
    }

    PointLayerModel::ConstPtr lp = LayerModel::as<PointLayerModel const>(l);
    if(lp) {
        p.setColor(Qt::black);

        std::vector<QPointF> points;
        lp->getPoints(points);
        for(const QPointF &p : points) {
            painter.moveTo(p);
            painter.addEllipse(p, 0.1, 0.1);
        }
    }

    QGraphicsPathItem *path = scene_->addPath(painter, p, b);
    paths_[layer_name] = path;
    path->setVisible(l->getVisibility());
}

void Renderer::doUpdate(const QString &layer_name)
{
    LayerModel::ConstPtr l = map_->getLayer(layer_name);
    QGraphicsPathItem *path = paths_[layer_name];

    QPen p = default_pen_;

    QColor color = l->getColor();
    p.setColor(color);
    color.setAlpha(127);
    QBrush b(color);
    PointLayerModel::ConstPtr lp = LayerModel::as<PointLayerModel const>(l);
    if(lp) {
        p.setColor(Qt::black);
    }
    path->setPen(p);
    path->setBrush(b);
    path->setVisible(l->getVisibility());

}


