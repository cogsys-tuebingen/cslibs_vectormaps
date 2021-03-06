#include "vector_layer_model.h"

#include <QGraphicsLineItem>
#include <QGraphicsItemGroup>

using namespace cslibs_vectormaps;

VectorLayerModel::VectorLayerModel()
{
}

VectorLayerModel::~VectorLayerModel()
{
}

void VectorLayerModel::setVectors(const dxf::DXFMap::Vectors &v)
{
    vectors_ = v;
}

void VectorLayerModel::getVectors(dxf::DXFMap::Vectors &v) const
{
    v = vectors_;
}

void VectorLayerModel::setVectors(const QLineFList &vectors)
{
    using ptype = dxf::DXFMap::Point;

    vectors_.clear();
    for(const auto &v : vectors) {
        const auto &p1 = v.p1();
        const auto &p2 = v.p2();
        vectors_.emplace_back(ptype(p1.x(), p1.y()), ptype(p2.x(), p2.y()));
    }
}

void VectorLayerModel::getVectors(QLineFList &vectors) const
{
    for(const auto &v : vectors_) {
        const auto &p1 = v.first;
        const auto &p2 = v.second;
        vectors.emplace_back(p1.x(), p1.y(), p2.x(), p2.y());
    }
}

QGraphicsItem* VectorLayerModel::render(const QPen& pen)
{
    QPen p(pen);
    p.setColor(getColor());
    std::vector<QLineF> lines;
    getVectors(lines);
    QGraphicsItemGroup *group = new QGraphicsItemGroup;
    for(const auto &l : lines) {
        QGraphicsLineItem *i = new QGraphicsLineItem(l);
        i->setPen(p);
        group->addToGroup(i);
    }
    return group;
}

void VectorLayerModel::update(QGraphicsItem &item, const QPen& pen)
{
    QPen p(pen);
    p.setColor(getColor());
    QGraphicsItemGroup& group = static_cast<QGraphicsItemGroup&>(item);
    QList<QGraphicsItem*> children = group.childItems();
    for(auto *child : children) {
        static_cast<QGraphicsLineItem*>(child)->setPen(p);
    }
}
