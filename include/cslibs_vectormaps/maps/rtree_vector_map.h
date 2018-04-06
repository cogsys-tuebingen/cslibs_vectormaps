#ifndef RTREE_VECTOR_MAP_H
#define RTREE_VECTOR_MAP_H

#include "vector_map.h"

#include <boost/geometry/index/rtree.hpp>

#include <string>
#include <tuple>

namespace cslibs_vectormaps {

class RtreeVectorMap : public VectorMap {
protected:
    using box_t = boost::geometry::model::box<VectorMap::Point>;
    using ring_t = boost::geometry::model::ring<VectorMap::Point>;
    using polygon_t = boost::geometry::model::polygon<VectorMap::Point>;
    using cell_t = std::tuple<box_t, std::vector<const Vector*>, double>;

public:
    typedef std::shared_ptr<RtreeVectorMap> Ptr;

    RtreeVectorMap(const BoundingBox& bounding, bool debug);

    virtual ~RtreeVectorMap();

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

    void insert(const Vectors& segments,
                const std::vector<polygon_t>& room_polygons,
                const std::vector<std::vector<std::size_t>>& segment_indices);

    unsigned int handleInsertion() override;
    void doLoad(const YAML::Node& node) override;
    void doSave(YAML::Node& node) const override;

protected:
    boost::geometry::index::rtree<cell_t, boost::geometry::index::rstar<16>> rtree_;
    std::vector<polygon_t> room_polygons_;
};

}

#endif // RTREE_VECTOR_MAP_H
