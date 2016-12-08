#include "layer_model.h"

using namespace utils_gdal;

LayerModel::LayerModel() :
    visible_(true),
    color_(Qt::black)
{

}

void LayerModel::setVisible(const bool visbile)
{
    visible_ = visbile;
}

bool LayerModel::getVisibility() const
{
    return visible_;
}

void LayerModel::setName(const std::string &name)
{
    name_ = name;
}

void LayerModel::getName(std::string &name) const
{
    name = name_;
}

void LayerModel::setVectors(const dxf::DXFMap::Vectors &v)
{
    vectors_ = v;
}

void LayerModel::getVectors(dxf::DXFMap::Vectors &v) const
{
    v = vectors_;
}

void LayerModel::setPoints(const dxf::DXFMap::Points &p)
{
    points_ = p;
}

void LayerModel::getPoints(dxf::DXFMap::Points &p)
{
    p = points_;
}

void LayerModel::setName(const QString &name)
{
    name_ = name.toStdString();
}

void LayerModel::getName(QString &name) const
{
    name = QString(name_.c_str());
}

void LayerModel::setVectors(const QLineFList &vectors)
{
    using ptype = dxf::DXFMap::Point;
    using vtype = dxf::DXFMap::Vector;

    vectors_.clear();
    for(const auto &v : vectors) {
        const auto &p1 = v.p1();
        const auto &p2 = v.p2();

        vectors_.emplace_back(vtype(ptype(p1.x(), p1.y()),
                                    ptype(p2.x(), p2.y())));

    }
}

void LayerModel::getVectors(QLineFList &vectors) const
{
    for(const auto &v : vectors_)
    {
        const auto &p1 = v.first;
        const auto &p2 = v.second;
        vectors.emplace_back(QLineF(p1.x(), p1.y(), p2.x(), p2.y()));
    }
}

void LayerModel::setPoints(const QPointFList &points)
{
    using ptype = dxf::DXFMap::Point;

    points_.clear();
    for(auto &p : points)
        points_.emplace_back(ptype(p.x(), p.y()));
}

void LayerModel::getPoints(QPointFList &points) const
{
    for(const auto &p : points_)
        points.emplace_back(QPointF(p.x(), p.y()));
}

void LayerModel::setColor(const QColor &color)
{
    color_ = color;
}

QColor LayerModel::getColor() const
{
    return color_;
}
