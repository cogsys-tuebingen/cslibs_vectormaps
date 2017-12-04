#ifndef SIMPLE_GRID_VECTOR_MAP_H
#define SIMPLE_GRID_VECTOR_MAP_H

#include "grid_vector_map.h"

namespace cslibs_vectormaps {
class SimpleGridVectorMap : public GridVectorMap
{
public:
    SimpleGridVectorMap(const BoundingBox &bounding,
                        const double range,
                        const double resolution,
                        const bool   debug = false);

    SimpleGridVectorMap();

    virtual ~SimpleGridVectorMap();


    virtual bool   structureNearby(const Point &pos,
                                   const double thresh = 0.15) const;

    virtual double minDistanceNearbyStructure(const Point &pos) const;

    virtual double minSquaredDistanceNearbyStructure(const Point &pos) const;

    virtual bool   retrieveFiltered(const Point &pos,
                                    Vectors &lines) const;

    virtual bool   retrieve(const Point &pos,
                            Vectors &lines) const;

    virtual int    intersectScanPattern(const Point   &position,
                                        const Vectors &pattern,
                                        IntersectionSet& intersections) const;

    virtual void   intersectScanPattern(const Point &position,
                                        const Vectors &pattern,
                                        std::vector<float> &ranges,
                                        const float default_mesaurement = 0.f) const;

protected:
    virtual unsigned int handleInsertion();
    virtual void doLoad(const YAML::Node &node);
    virtual void doSave(YAML::Node &node) const;

};
}

#endif // SIMPLE_GRID_VECTOR_MAP_H
