#include "point_layer_model.h"

#include <QBrush>

using namespace cslibs_vectormaps;

PointLayerModel::PointLayerModel()
{

}

PointLayerModel::~PointLayerModel()
{

}

void PointLayerModel::setPoints(const dxf::DXFMap::Points &points)
{
    points_ = points;
}

void PointLayerModel::getPoints(dxf::DXFMap::Points &p) const
{
    p = points_;
}

void PointLayerModel::setPoints(const QPointFList &points)
{
    using ptype = dxf::DXFMap::Point;

    points_.clear();
    for(auto &p : points)
        points_.emplace_back(ptype(p.x(), p.y()));
}

void PointLayerModel::getPoints(QPointFList &points) const
{
    for(const auto &p : points_)
        points.emplace_back(QPointF(p.x(), p.y()));
}

void PointLayerModel::render(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const
{
    QPen p(pen);
    p.setColor(Qt::black);
    QColor c(getColor());
    c.setAlphaF(point_alpha);
    QBrush b(c);

    std::vector<QPointF> points;
    getPoints(points);
    for(const QPointF &point : points) {
        QGraphicsEllipseItem *i = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        i->setPen(p);
        i->setBrush(b);
        group.addToGroup(i);
    }
}

void PointLayerModel::update(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const
{
    QColor c(getColor());
    c.setAlphaF(point_alpha);
    QBrush b(c);
    QList<QGraphicsItem*> children = group.childItems();
    for(auto *child : children) {
        static_cast<QGraphicsEllipseItem*>(child)->setBrush(b);
    }
}
