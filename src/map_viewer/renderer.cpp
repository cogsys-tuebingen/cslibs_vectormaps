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

    QGraphicsItemGroup *g = new QGraphicsItemGroup;
    l->render(*g, default_pen_, default_point_alpha_);
    groups_[layer_name] = g;
    g->setVisible(l->getVisibility());

    add(g);
}

void Renderer::doUpdate(const QString &layer_name)
{
    LayerModel::ConstPtr l = map_->getLayer(layer_name);
    QGraphicsItemGroup *g = groups_[layer_name];

    l->update(*g, default_pen_, default_point_alpha_);
    g->setVisible(l->getVisibility());
}


