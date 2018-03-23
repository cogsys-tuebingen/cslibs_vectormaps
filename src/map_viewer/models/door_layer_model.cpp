#include "door_layer_model.h"

#include <Qt>

using namespace cslibs_vectormaps;

DoorLayerModel::DoorLayerModel() : PolygonLayerModel(1.0)
{
    setColor(Qt::red);
}

DoorLayerModel::~DoorLayerModel()
{
}

void DoorLayerModel::setDoor(const FindDoors::door_t& door)
{
    door_ = door;
}

void DoorLayerModel::getDoor(FindDoors::door_t& door) const
{
    door = door_;
}
