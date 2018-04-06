#include "polygon_layer_model.h"

#include <QBrush>
#include <QAbstractGraphicsShapeItem>

using namespace cslibs_vectormaps;

PolygonLayerModel::PolygonLayerModel(double alpha) : alpha_(alpha)
{
}

PolygonLayerModel::~PolygonLayerModel()
{
}

void PolygonLayerModel::setPolygon(const polygon_t& polygon)
{
    polygon_ = polygon;
}

void PolygonLayerModel::getPolygon(polygon_t& polygon) const
{
    polygon = polygon_;
}

QGraphicsItem* PolygonLayerModel::render(const QPen& pen)
{
    QPolygonF outerpolygon;
    for (const auto& p : polygon_.outer())
        outerpolygon.push_back(QPointF(p.x(), p.y()));

    QGraphicsItem* item;
    if (polygon_.inners().size() == 0) {
        PolygonItem* polygonitem = new PolygonItem(outerpolygon, *this);
        item = polygonitem;
    } else {
        QPainterPath path;
        path.addPolygon(outerpolygon);
        for (const ring_t& hole : polygon_.inners()) {
            QPainterPath inner;
            //add inner points
            QPolygonF innerpolygon;
            for (const auto& p2 : hole)
                innerpolygon.push_back(QPointF(p2.x(), p2.y()));
            inner.addPolygon(innerpolygon);
            path = path.subtracted(inner);
        }
        PolygonItemWithHoles* polygonitem = new PolygonItemWithHoles(path, *this);
        item = polygonitem;
    }

    update(*item, pen);
    return item;
}

void PolygonLayerModel::update(QGraphicsItem& item, const QPen& pen)
{
    QColor c(getColor());
    QPen p(pen);
    p.setColor(c);
    c.setAlphaF(alpha_);
    QBrush b(c);
    QAbstractGraphicsShapeItem& shapeitem = static_cast<QAbstractGraphicsShapeItem&>(item);
    shapeitem.setPen(p);
    shapeitem.setBrush(b);
}

PolygonItem::PolygonItem(const QPolygonF& qpolygonf, PolygonLayerModel& model)
    : QGraphicsPolygonItem(qpolygonf), model_(model)
{
}

PolygonItemWithHoles::PolygonItemWithHoles(const QPainterPath& qpainterpath, PolygonLayerModel& model)
    : QGraphicsPathItem(qpainterpath), model_(model)
{
}
