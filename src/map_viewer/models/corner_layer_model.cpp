#include "corner_layer_model.h"

#include <QBrush>
#include <QGraphicsEllipseItem>
#include <QGraphicsItemGroup>

using namespace cslibs_vectormaps;

CornerLayerModel::CornerLayerModel(double alpha) : PointLayerModel(alpha)
{
}

CornerLayerModel::~CornerLayerModel()
{
}

void CornerLayerModel::setPoints(const dxf::DXFMap::Points &points)
{
    PointLayerModel::setPoints(points);
    cornerness_.resize(points_.size(), 0);
}

void CornerLayerModel::setPoints(const QPointFList &points)
{
    PointLayerModel::setPoints(points);
    cornerness_.resize(points_.size(), 0);
}

void CornerLayerModel::setCornerness(const std::vector<double> &cornerness)
{
    cornerness_ = cornerness;
}

void CornerLayerModel::getCornerness(std::vector<double> &cornerness) const
{
    cornerness = cornerness_;
}

QGraphicsItem* CornerLayerModel::render(const QPen& pen)
{
    QColor color = getColor();
    QPen p(pen);
    p.setColor(Qt::black);

    std::vector<double> cornerness;
    getCornerness(cornerness);

    std::vector<QPointF> points;
    getPoints(points);

    QGraphicsItemGroup *group = new QGraphicsItemGroup;
    for(std::size_t i = 0; i < points.size(); ++i) {
        const QPointF &point = points[i];
        QGraphicsEllipseItem *item = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        const double &corner = cornerness[i];
        color.setAlphaF(corner * alpha_);
        QBrush b(color);
        item->setBrush(b);
        item->setPen(p);
        group->addToGroup(item);
    }
    return group;
}

void CornerLayerModel::update(QGraphicsItem &item, const QPen& pen)
{
    QColor color = getColor();
    QPen p(pen);
    p.setColor(Qt::black);

    std::vector<double> cornerness;
    getCornerness(cornerness);

    QGraphicsItemGroup& group = static_cast<QGraphicsItemGroup&>(item);
    QList<QGraphicsItem*> children = group.childItems();
    for(std::size_t i = 0; i < children.size(); ++i) {
        QGraphicsEllipseItem *item = static_cast<QGraphicsEllipseItem*>(children[i]);
        const double &corner = cornerness[i];
        color.setAlphaF(corner * alpha_);
        QBrush b(color);
        item->setBrush(b);
        item->setPen(p);
    }
}
