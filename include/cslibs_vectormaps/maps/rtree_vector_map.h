#ifndef RTREE_VECTOR_MAP_H
#define RTREE_VECTOR_MAP_H

#include "vector_map.h"

#include <string>

namespace cslibs_vectormaps {

class RtreeVectorMap : public VectorMap {
public:
    RtreeVectorMap(const BoundingBox& bounding, bool debug);

    virtual ~RtreeVectorMap();

    virtual bool   structureNearby(const Point &pos,
                                   const double thresh = 0.15) const final;

    virtual double minDistanceNearbyStructure(const Point &pos) const final;

    virtual double minSquaredDistanceNearbyStructure(const Point &pos) const final;

    virtual bool   retrieveFiltered(const Point &pos,
                                    Vectors &lines) const final;

    virtual bool   retrieve(const Point &pos,
                            Vectors &lines) const final;

    virtual int    intersectScanPattern(const Point   &position,
                                        const Vectors &pattern,
                                        IntersectionSet& intersections) const final;

    virtual void   intersectScanPattern(const Point &position,
                                        const Vectors &pattern,
                                        std::vector<float> &ranges,
                                        const float default_mesaurement = 0.f) const final;

    virtual unsigned int handleInsertion();
    virtual void doLoad(const YAML::Node& node);
    virtual void doSave(YAML::Node& node) const;
};

}

#endif // RTREE_VECTOR_MAP_H
