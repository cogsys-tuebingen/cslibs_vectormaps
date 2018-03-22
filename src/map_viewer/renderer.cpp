#include "renderer.h"

#include "map.h"
#include "models/layer_model.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItemGroup>

using namespace cslibs_vectormaps;

Renderer::Renderer() :
    stop_(false)
{
}

Renderer::~Renderer()
{
    {
        std::lock_guard<std::mutex> l(render_queue_mutex_);
        stop_ = true;
    }
    render_condition_.notify_one();
    worker_thread_.join();

    delete scene_;
}

void Renderer::setup(Map *map, QGraphicsView *graphics_view)
{
    map_ = map;
    view_ = graphics_view;
    scene_ = new QGraphicsScene();
    view_->setOptimizationFlags(QGraphicsView::DontSavePainterState);

    view_->setScene(scene_);

    connect(this, SIGNAL(finished()), this, SLOT(postRendering()), Qt::QueuedConnection);
    connect(this, SIGNAL(clear()), this, SLOT(clearScene()), Qt::QueuedConnection);
    connect(this, SIGNAL(add(QGraphicsItem*)), this, SLOT(addItem(QGraphicsItem*)), Qt::QueuedConnection);

    worker_thread_ = std::thread(&Renderer::run, this);
}

void Renderer::setDefaultPen(const QPen &pen)
{
    default_pen_ = pen;
}

void Renderer::run()
{
    std::unique_lock<std::mutex> l(render_queue_mutex_);
    while(!stop_) {
        while(render_queue_.empty() && !stop_)
            render_condition_.wait(l);

        if(!render_queue_.empty()) {
            render_queue_.front()();
            render_queue_.pop();
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

void Renderer::addItem(QGraphicsItem *g)
{
    scene_->addItem(g);
}

void Renderer::doRepaint()
{
    clear();
    items_.clear();

    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    for(auto &l : layers) {
        doRepaint(l->getName<QString>());
    }
}

void Renderer::doRepaint(const QString &layer_name)
{
    LayerModel::Ptr l = map_->getLayer(layer_name);

    QGraphicsItem *i = l->render(default_pen_);
    items_[layer_name] = i;
    i->setVisible(l->getVisibility());

    add(i);
}

void Renderer::doUpdate(const QString &layer_name)
{
    LayerModel::Ptr l = map_->getLayer(layer_name);
    QGraphicsItem *i = items_[layer_name];

    l->update(*i, default_pen_);
    i->setVisible(l->getVisibility());
}
