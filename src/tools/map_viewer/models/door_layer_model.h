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

};
}

#endif // DOORLAYERMODEL_H
