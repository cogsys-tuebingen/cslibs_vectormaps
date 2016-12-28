#ifndef POINTLAYERMODEL_H
#define POINTLAYERMODEL_H

#include "layer_model.h"

namespace cslibs_gdal {
class PointLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<PointLayerModel>        Ptr;
    typedef std::shared_ptr<PointLayerModel const>  ConstPtr;
    typedef std::vector<QPointF>                    QPointFList;

    PointLayerModel();

    virtual ~PointLayerModel();

    static inline LayerModel::Ptr asBase(const Ptr &layer)
    {
        return std::dynamic_pointer_cast<LayerModel>(layer);
    }

    static inline LayerModel::ConstPtr asBase(const ConstPtr &layer)
    {
        return std::dynamic_pointer_cast<LayerModel const>(layer);
    }

    void setPoints(const dxf::DXFMap::Points &p);

    void getPoints(dxf::DXFMap::Points &p) const;

    void setPoints(const QPointFList &vectors);

    void getPoints(QPointFList &points) const;

private:
    dxf::DXFMap::Points points_;

};
}

#endif // POINTLAYERMODEL_H
