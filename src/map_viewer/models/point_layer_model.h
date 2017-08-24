#ifndef POINTLAYERMODEL_H
#define POINTLAYERMODEL_H

#include "layer_model.h"

namespace cslibs_vectormaps {
class PointLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<PointLayerModel>        Ptr;
    typedef std::shared_ptr<PointLayerModel const>  ConstPtr;
    typedef std::vector<QPointF>                    QPointFList;

    PointLayerModel();

    virtual ~PointLayerModel();

    virtual void setPoints(const dxf::DXFMap::Points &points);

    void getPoints(dxf::DXFMap::Points &p) const;

    virtual void setPoints(const QPointFList &points);

    void getPoints(QPointFList &points) const;

protected:
    dxf::DXFMap::Points points_;

};
}

#endif // POINTLAYERMODEL_H
