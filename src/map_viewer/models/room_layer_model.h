#ifndef ROOMLAYERMODEL_H
#define ROOMLAYERMODEL_H

#include "polygon_layer_model.h"

namespace cslibs_vectormaps {

class RoomLayerModel : public PolygonLayerModel {
public:
    typedef std::shared_ptr<RoomLayerModel>       Ptr;
    typedef std::shared_ptr<RoomLayerModel const> ConstPtr;

    RoomLayerModel();
    virtual ~RoomLayerModel();
};

}

#endif // DOORLAYERMODEL_H
