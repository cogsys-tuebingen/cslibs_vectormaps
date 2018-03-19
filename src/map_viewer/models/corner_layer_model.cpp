#include "corner_layer_model.h"

using namespace cslibs_vectormaps;

CornerLayerModel::CornerLayerModel()
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

void CornerLayerModel::render(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const
{
    QColor color = getColor();
    QPen p(pen);
    p.setColor(Qt::black);

    std::vector<QPointF> points;
    std::vector<double>  cornerness;
    getPoints(points);
    getCornerness(cornerness);

    for(std::size_t i = 0; i < points.size(); ++i) {
        const QPointF &point = points[i];
        const double &corner = cornerness[i];
        color.setAlphaF(corner * point_alpha);
        QBrush b(color);
        QGraphicsEllipseItem *item = new QGraphicsEllipseItem(point.x() - 0.1, point.y() - 0.1, 0.2, 0.2);
        item->setBrush(b);
        item->setPen(p);
        group.addToGroup(item);
    }
}

void CornerLayerModel::update(QGraphicsItemGroup &group, const QPen& pen, double point_alpha) const
{
    QColor color = getColor();
    QPen p(pen);
    p.setColor(Qt::black);

    std::vector<double>  cornerness;
    getCornerness(cornerness);

    QList<QGraphicsItem*> children = group.childItems();
    assert(cornerness.size() == children.size());
    for(std::size_t i = 0; i < children.size(); ++i) {
        QGraphicsEllipseItem *item = static_cast<QGraphicsEllipseItem*>(children[i]);
        const double &corner = cornerness[i];
        color.setAlphaF(corner * point_alpha);
        QBrush b(color);
        item->setBrush(b);
        item->setPen(p);
    }
}
