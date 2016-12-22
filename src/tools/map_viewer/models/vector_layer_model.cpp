#include "vector_layer_model.h"

using namespace utils_gdal;

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
    using vtype = dxf::DXFMap::Vector;

    vectors_.clear();
    for(const auto &v : vectors) {
        const auto &p1 = v.p1();
        const auto &p2 = v.p2();

        vectors_.emplace_back(vtype(ptype(p1.x(), p1.y()),
                                    ptype(p2.x(), p2.y())));

    }
}

void VectorLayerModel::getVectors(QLineFList &vectors) const
{
    for(const auto &v : vectors_)
    {
        const auto &p1 = v.first;
        const auto &p2 = v.second;
        vectors.emplace_back(QLineF(p1.x(), p1.y(), p2.x(), p2.y()));
    }
}
