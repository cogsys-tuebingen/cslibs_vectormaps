#include "polygon_layer_model.h"

#include <QBrush>
#include <QPolygonF>
#include <QGraphicsPolygonItem>

using namespace cslibs_vectormaps;

PolygonLayerModel::PolygonLayerModel()
{
}

PolygonLayerModel::~PolygonLayerModel()
{
}

void PolygonLayerModel::setPolygon(const dxf::DXFMap::Points& polygon)
{
    polygon_ = polygon;
}

void PolygonLayerModel::getPolygon(dxf::DXFMap::Points& polygon) const
{
    polygon = polygon_;
}

void PolygonLayerModel::setPolygon(const QVector<QPointF>& polygon)
{
    polygon_.clear();
    for (const auto& p : polygon)
        polygon_.emplace_back(p.x(), p.y());
}

void PolygonLayerModel::getPolygon(QVector<QPointF>& polygon) const
{
    for (const auto& p : polygon_)
        polygon.push_back(QPointF(p.x(), p.y()));
}

void PolygonLayerModel::render(QGraphicsItemGroup& group, const QPen& pen, double point_alpha) const
{
    QPen p(pen);
    p.setColor(getColor());
    QVector<QPointF> polygon_points;
    getPolygon(polygon_points);
    QPolygonF qpolygon(polygon_points);
    QGraphicsPolygonItem* i = new QGraphicsPolygonItem(qpolygon);
    i->setPen(p);
    group.addToGroup(i);
}

void PolygonLayerModel::update(QGraphicsItemGroup& group, const QPen& pen, double point_alpha) const
{
    QColor c(getColor());
    QPen p(pen);
    p.setColor(c);
    c.setAlphaF(point_alpha);
    QBrush b(c);
    QList<QGraphicsItem*> children = group.childItems();
    for(auto* child : children) {
        static_cast<QGraphicsPolygonItem*>(child)->setPen(p);
        static_cast<QGraphicsPolygonItem*>(child)->setBrush(b);
    }
}
