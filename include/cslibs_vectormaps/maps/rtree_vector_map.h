#ifndef RTREE_VECTOR_MAP_H
#define RTREE_VECTOR_MAP_H

#include "vector_map.h"

#include <boost/geometry/index/rtree.hpp>

#include <string>
#include <utility>

namespace cslibs_vectormaps {

class RtreeVectorMap : public VectorMap {
protected:
    using box_t = boost::geometry::model::box<VectorMap::Point>;
    using ring_t = boost::geometry::model::ring<VectorMap::Point>;

public:
    typedef std::shared_ptr<RtreeVectorMap> Ptr;

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

    void insert(const Vectors& segments,
                const std::vector<ring_t>& rooms,
                const std::vector<std::vector<std::size_t>>& segment_indices);

    virtual unsigned int handleInsertion();
    virtual void doLoad(const YAML::Node& node);
    virtual void doSave(YAML::Node& node) const;

protected:
    boost::geometry::index::rtree<std::pair<box_t, std::vector<const Vector*>>, boost::geometry::index::rstar<16>> rtree_;
    std::vector<ring_t> room_rings_;
};

}

#endif // RTREE_VECTOR_MAP_H
