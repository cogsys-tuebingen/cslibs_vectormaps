#include "corner_layer_model.h"

using namespace cslibs_gdal;

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
