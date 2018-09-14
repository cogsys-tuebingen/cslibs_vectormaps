// Apparently, indexing line segments is only supported since Boost.Geometry
// 1.56. Only for code compatibility, an empty implementation of
// SegmentRtreeVectorMap is provided that does not do anything.

#ifndef SEGMENT_RTREE_VECTOR_MAP_H
#define SEGMENT_RTREE_VECTOR_MAP_H

#include "vector_map.h"

#include <boost/version.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <string>
#include <tuple>

namespace cslibs_vectormaps {

class SegmentRtreeVectorMap : public VectorMap {
protected:
#if BOOST_VERSION >= 105600
    using tree_t = boost::geometry::index::rtree<Vector, boost::geometry::index::rstar<8>>;
#else
    using tree_t = int; // dummy
#endif

public:
    typedef std::shared_ptr<SegmentRtreeVectorMap> Ptr;

    SegmentRtreeVectorMap();

    SegmentRtreeVectorMap(const BoundingBox& bounding, bool debug);

    virtual ~SegmentRtreeVectorMap();

    const void* cell(const Point& pos) const override;

    bool   structureNearby(const Point &pos,
                           const double thresh = 0.15) const override;

    double minSquaredDistanceNearbyStructure(const Point& pos,
                                             const void* cell_ptr,
                                             double angle) const override;

    double minDistanceNearbyStructure(const Point &pos) const override;

    double minSquaredDistanceNearbyStructure(const Point &pos) const override;

    bool   retrieveFiltered(const Point &pos,
                            Vectors &lines) const override;

    bool   retrieve(const Point &pos,
                    Vectors &lines) const override;

    double intersectScanRay(const Vector& ray,
                            const void* cell_ptr,
                            double angle,
                            double max_range) const override;

    int    intersectScanPattern(const Point   &position,
                                const Vectors &pattern,
                                IntersectionSet& intersections) const override;

    void   intersectScanPattern(const Point &position,
                                const Vectors &pattern,
                                std::vector<float> &ranges,
                                const float default_mesaurement = 0.f) const override;

    void insert(const Vectors& segments);

    unsigned int handleInsertion() override;
    void doLoad(const YAML::Node& node) override;
    void doSave(YAML::Node& node) const override;

    const tree_t& rtree() const;

protected:
    tree_t rtree_;
};

}

#endif // SEGMENT_RTREE_VECTOR_MAP_H
