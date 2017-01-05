#ifndef DOORLAYERMODEL_H
#define DOORLAYERMODEL_H

#include "layer_model.h"

namespace cslibs_gdal {
class DoorLayerModel : public LayerModel
{
public:
    typedef std::shared_ptr<DoorLayerModel>        Ptr;
    typedef std::shared_ptr<DoorLayerModel const>  ConstPtr;

    DoorLayerModel();
    virtual ~DoorLayerModel();

    static inline LayerModel::Ptr asBase(const Ptr &layer)
    {
        return std::dynamic_pointer_cast<LayerModel>(layer);
    }

    static inline LayerModel::ConstPtr asBase(const ConstPtr &layer)
    {
        return std::dynamic_pointer_cast<LayerModel const>(layer);
    }


};
}

#endif // DOORLAYERMODEL_H
