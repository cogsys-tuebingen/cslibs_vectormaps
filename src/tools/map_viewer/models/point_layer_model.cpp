#include "point_layer_model.h"

using namespace utils_gdal;

PointLayerModel::PointLayerModel()
{

}

PointLayerModel::~PointLayerModel()
{

}

void PointLayerModel::setPoints(const dxf::DXFMap::Points &p)
{
    points_ = p;
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
