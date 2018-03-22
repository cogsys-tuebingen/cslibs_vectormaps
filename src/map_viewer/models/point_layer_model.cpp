#include "point_layer_model.h"

#include <QBrush>
#include <QGraphicsEllipseItem>
#include <QGraphicsItemGroup>

using namespace cslibs_vectormaps;

PointLayerModel::PointLayerModel(double alpha) : alpha_(alpha)
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

QGraphicsItem* PointLayerModel::render(const QPen& pen)
{
    QPen p(pen);
    p.setColor(Qt::black);
    QColor c(getColor());
    c.setAlphaF(alpha_);
    QBrush b(c);

    std::vector<QPointF> points;
    getPoints(points);
    QGraphicsItemGroup *group = new QGraphicsItemGroup;
    for(const QPointF &point : points) {
        QGraphicsEllipseItem *i = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        i->setPen(p);
        i->setBrush(b);
        group->addToGroup(i);
    }
    return group;
}

void PointLayerModel::update(QGraphicsItem &item, const QPen& pen)
{
    QPen p(pen);
    p.setColor(Qt::black);
    QColor c(getColor());
    c.setAlphaF(alpha_);
    QBrush b(c);

    QGraphicsItemGroup& group = static_cast<QGraphicsItemGroup&>(item);
    QList<QGraphicsItem*> children = group.childItems();
    for(auto *child : children) {
        static_cast<QGraphicsEllipseItem*>(child)->setPen(p);
        static_cast<QGraphicsEllipseItem*>(child)->setBrush(b);
    }
}
