// Apparently, indexing line segments is only supported since Boost.Geometry
// 1.56. Only for code compatibility, an empty implementation of
// SegmentRtreeVectorMap is provided that does not do anything.

#include <cslibs_vectormaps/maps/segment_rtree_vector_map.h>

#include <cslibs_vectormaps/utility/serialization.hpp>
#include <cslibs_boost_geometry/algorithms.h>

#include <boost/version.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/comparable_distance.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#if BOOST_VERSION < 105500
#include <boost/function_output_iterator.hpp>
#endif

#include <limits>
#include <cstdint>

using namespace cslibs_vectormaps;
using namespace cslibs_boost_geometry;

SegmentRtreeVectorMap::SegmentRtreeVectorMap()
{

}

SegmentRtreeVectorMap::SegmentRtreeVectorMap(const BoundingBox& bounding, bool debug) :
    VectorMap(bounding, 123456789., debug) // TODO
{

}

SegmentRtreeVectorMap::~SegmentRtreeVectorMap()
{
}

const void* SegmentRtreeVectorMap::cell(const Point& pos) const
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    double max_area_ratio = 0.;
    const void* cell_ptr = nullptr;

    return nullptr;
}

double SegmentRtreeVectorMap::minSquaredDistanceNearbyStructure(const Point& pos,
                                                                const void* cell_ptr,
                                                                double angle) const
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    double min_squared_dist = std::numeric_limits<double>::max();

#if BOOST_VERSION >= 105600
    for (auto it = rtree_.qbegin(bgi::nearest(pos, 1)), end = rtree_.qend(); it != end; ++it) {
        const Vector& segment = *it;
        min_squared_dist = bg::comparable_distance(pos, segment);
    }
#endif

    return min_squared_dist;
}

double SegmentRtreeVectorMap::minDistanceNearbyStructure(const Point& pos) const
{
    return 0.;
}

double SegmentRtreeVectorMap::minSquaredDistanceNearbyStructure(const Point& pos) const
{
    return 0.;
}

bool SegmentRtreeVectorMap::structureNearby(const Point& pos,
                                            const double thresh) const
{
    return false;
}

bool SegmentRtreeVectorMap::retrieveFiltered(const Point& pos,
                                             Vectors& lines) const
{
    return false;
}

bool SegmentRtreeVectorMap::retrieve(const Point& pos,
                                     Vectors& lines) const
{
    return false;
}

double SegmentRtreeVectorMap::intersectScanRay(const Vector& ray,
                                               const void* cell_ptr,
                                               double angle,
                                               double max_range) const
{
#if BOOST_VERSION >= 105600
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    // Boost.Geometry has a nice path tracing feature, but it's still experimental as of 24.04.2018
#ifdef BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
    double dist = max_range;
    for (auto it = rtree_.qbegin(bgi::path(ray, 1)), end = rtree_.qend(); it != end; ++it) {
        const Vector& segment = *it;
        std::vector<Point> tmp;
        bg::intersection(ray, segment, tmp);
        dist = bg::distance(ray.first, tmp.front());
    }
    return dist;
#else
    // the workaround is to use intersects(), so we still have to find out the closest segment
    double min_squared_dist = std::numeric_limits<double>::max();
    std::vector<Point> tmp;
    for (auto it = rtree_.qbegin(bgi::intersects(ray)), end = rtree_.qend(); it != end; ++it) {
        const Vector& segment = *it;
        tmp.clear();
        bg::intersection(ray, segment, tmp);
        double squared_dist = bg::comparable_distance(ray.first, tmp.front());
        if (squared_dist < min_squared_dist)
            min_squared_dist = squared_dist;
    }
    return min_squared_dist != std::numeric_limits<double>::max()
           ? std::sqrt(min_squared_dist)
           : max_range;
#endif

#else
    return max_range;
#endif
}

int SegmentRtreeVectorMap::intersectScanPattern(
    const Point& pos,
    const Vectors& pattern,
    IntersectionSet& intersections) const
{
    return 0;
}

void SegmentRtreeVectorMap::intersectScanPattern(const Point& pos,
                                                 const Vectors& pattern,
                                                 std::vector<float>& ranges,
                                                 const float default_measurement) const
{

}

// The map is newly created using the given data.
void SegmentRtreeVectorMap::insert(const Vectors& segments)
{
    data_ = segments;

    // We bulk-insert into the R-tree so that the efficient packing algorithm is
    // used. All values have to be passed to the constructor at once.
#if BOOST_VERSION >= 105600
    rtree_.~rtree();
    new (&rtree_) tree_t(data_);
#else
    rtree_ = 0;
#endif
}

unsigned int SegmentRtreeVectorMap::handleInsertion()
{
    return 0;
}

void SegmentRtreeVectorMap::doLoad(const YAML::Node& node)
{
    VectorMap::doLoad(node);
    insert(data_);
}

void SegmentRtreeVectorMap::doSave(YAML::Node& node) const
{
    VectorMap::doSave(node);
    node["map_type"] = "segment_rtree";
}

const SegmentRtreeVectorMap::tree_t& SegmentRtreeVectorMap::rtree() const
{
    return rtree_;
}
