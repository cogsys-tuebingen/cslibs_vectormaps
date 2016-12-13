#ifndef POINTLAYERMODEL_H
#define POINTLAYERMODEL_H

#include "layer_model.h"

namespace utils_gdal {
class PointLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<PointLayerModel>        Ptr;
    typedef std::shared_ptr<PointLayerModel const>  ConstPtr;
    typedef std::vector<QPointF>                    QPointFList;

    PointLayerModel();

    virtual ~PointLayerModel();

    void setPoints(const dxf::DXFMap::Vectors &v);

    void getPoints(dxf::DXFMap::Vectors &v) const;

    void setPoints(const QLineFList &vectors);

    void getPoints(QLineFList &vectors) const;

private:
    dxf::DXFMap::Points points_;

};
}

#endif // POINTLAYERMODEL_H
