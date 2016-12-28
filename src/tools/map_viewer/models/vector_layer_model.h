#ifndef VECTORLAYERMODEL_H
#define VECTORLAYERMODEL_H

#include "layer_model.h"

namespace cslibs_gdal {
class VectorLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<VectorLayerModel>        Ptr;
    typedef std::shared_ptr<VectorLayerModel const>  ConstPtr;
    typedef std::vector<QPointF>                     QPointFList;

    VectorLayerModel();
    virtual ~VectorLayerModel();

    static inline LayerModel::Ptr asBase(const Ptr &layer)
    {
        return std::dynamic_pointer_cast<LayerModel>(layer);
    }

    static inline LayerModel::ConstPtr asBase(const ConstPtr &layer)
    {
        return std::dynamic_pointer_cast<LayerModel const>(layer);
    }

    void setVectors(const dxf::DXFMap::Vectors &v);

    void getVectors(dxf::DXFMap::Vectors &v) const;

    void setVectors(const QLineFList &vectors);

    void getVectors(QLineFList &vectors) const;

private:
    dxf::DXFMap::Vectors vectors_;

};
}

#endif // VECTORLAYERMODEL_H
