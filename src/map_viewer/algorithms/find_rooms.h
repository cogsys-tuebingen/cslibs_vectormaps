#ifndef FIND_ROOMS_H
#define FIND_ROOMS_H

#include "find_doors.h"

namespace cslibs_vectormaps {

struct FindRoomsParameter {
    const FindDoorsParameter* find_doors_parameter;
    FindDoors::graph_t* graph;
};

class FindRooms {
public:
    FindRooms(const FindRoomsParameter &parameter);

    std::vector<std::vector<point_t>> find_rooms(const std::vector<FindDoors::door_t>& doors);
private:
    const FindRoomsParameter& parameter_;
};

}

#endif
