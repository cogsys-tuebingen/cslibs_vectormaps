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

// workarounds for older Boost versions according to
// https://stackoverflow.com/a/27799356/1007605

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

#if BOOST_VERSION >= 105500
    for (auto it = rtree_.qbegin(bgi::nearest(pos, 1)), end = rtree_.qend(); it != end; ++it) {
        const Vector& segment = *it;
#else
    rtree_.query(bgi::nearest(pos, 1), boost::make_function_output_iterator([&](const Vector& segment) {
#endif
        min_squared_dist = bg::comparable_distance(pos, segment);
#if BOOST_VERSION >= 105500
    }
#else
    }));
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
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    // Boost.Geometry has a nice path tracing feature, but it's still experimental as of 24.04.2018
#ifdef BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
    double dist = max_range;
#if BOOST_VERSION >= 105500
    for (auto it = rtree_.qbegin(bgi::path(ray, 1)), end = rtree_.qend(); it != end; ++it) {
        const Vector& segment = *it;
#else
    rtree_.query(bgi::path(ray, 1), boost::make_function_output_iterator([&](const Vector& segment) {
#endif
        std::vector<Point> tmp;
        bg::intersection(ray, segment, tmp);
        dist = bg::distance(ray.first, tmp.front());
#if BOOST_VERSION >= 105500
    }
#else
    }));
#endif
    return dist;
#else
    // the workaround is to use intersects(), so we still have to find out the closest segment
    double min_squared_dist = std::numeric_limits<double>::max();
    std::vector<Point> tmp;
#if BOOST_VERSION >= 105500
    for (auto it = rtree_.qbegin(bgi::intersects(ray)), end = rtree_.qend(); it != end; ++it) {
        const Vector& segment = *it;
#else
    rtree_.query(bgi::intersects(ray), boost::make_function_output_iterator([&](const Vector& segment) {
#endif
        tmp.clear();
        bg::intersection(ray, segment, tmp);
        double squared_dist = bg::comparable_distance(ray.first, tmp.front());
        if (squared_dist < min_squared_dist)
            min_squared_dist = squared_dist;
#if BOOST_VERSION >= 105500
    }
#else
    }));
#endif
    return min_squared_dist != std::numeric_limits<double>::max()
           ? std::sqrt(min_squared_dist)
           : max_range;
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
    rtree_.~rtree();
    new (&rtree_) tree_t(data_);
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
