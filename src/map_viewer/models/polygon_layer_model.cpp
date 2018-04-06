#include "polygon_layer_model.h"

#include <QBrush>
#include <QPolygonF>

using namespace cslibs_vectormaps;

PolygonLayerModel::PolygonLayerModel(double alpha) : alpha_(alpha)
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

QGraphicsItem* PolygonLayerModel::render(const QPen& pen)
{
    ConsciousPolygonItem* i = new ConsciousPolygonItem(*this);

    QVector<QPointF> polygon_points;
    getPolygon(polygon_points);
    QPolygonF qpolygon(polygon_points);
    i->setPolygon(qpolygon);

    update(*i, pen);
    return i;
}

void PolygonLayerModel::update(QGraphicsItem& item, const QPen& pen)
{
    QColor c(getColor());
    QPen p(pen);
    p.setColor(c);
    c.setAlphaF(alpha_);
    QBrush b(c);
    QGraphicsPolygonItem& polygon = static_cast<QGraphicsPolygonItem&>(item);
    polygon.setPen(p);
    polygon.setBrush(b);
}

ConsciousPolygonItem::ConsciousPolygonItem(PolygonLayerModel& model) : model_(model)
{
}
