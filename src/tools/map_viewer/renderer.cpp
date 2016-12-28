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

Renderer::Renderer()
{
}

void Renderer::setup(View *view, Map *map, QGraphicsView *grahpics_view)
{
    map_ = map;
    view_ = grahpics_view;
    scene_ = new QGraphicsScene(view_);
    view_->setOptimizationFlags(QGraphicsView::DontSavePainterState);

    view_->setScene(scene_);

    /// connect signals

}

void Renderer::setDefaultPen(const QPen &pen)
{
    default_pen_ = pen;
}

void Renderer::repaint()
{
    std::vector<LayerModel::Ptr> layers;
    map_->getLayers(layers);

    /// create new path items
    for(auto &l : layers) {
        repaint(l->getName<QString>());
    }

    view_->show();
}

void Renderer::repaint(const QString &layer_name)
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

void Renderer::update(const QString &layer_name)
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
    view_->update();
}
