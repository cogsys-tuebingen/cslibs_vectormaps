#ifndef DOORLAYERMODEL_H
#define DOORLAYERMODEL_H

#include "polygon_layer_model.h"

#include "../algorithms/find_doors.h"

namespace cslibs_vectormaps {

class DoorLayerModel : public PolygonLayerModel {
public:
    typedef std::shared_ptr<DoorLayerModel>       Ptr;
    typedef std::shared_ptr<DoorLayerModel const> ConstPtr;

    DoorLayerModel();
    virtual ~DoorLayerModel();

    void setDoor(const FindDoors::door_t& polygon);
    void getDoor(FindDoors::door_t& polygon) const;
protected:
    FindDoors::door_t door_;
};

}

#endif // DOORLAYERMODEL_H
