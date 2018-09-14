#include <cslibs_vectormaps/maps/rtree_vector_map.h>

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

RtreeVectorMap::RtreeVectorMap()
{

}

RtreeVectorMap::RtreeVectorMap(const BoundingBox& bounding, bool debug) :
    VectorMap(bounding, 123456789., debug) // TODO
{

}

RtreeVectorMap::~RtreeVectorMap()
{
}

namespace boost { namespace geometry { namespace index {

// indirect R-tree indexing (see "Specializing index::indexable function object" in docs)
template <typename Segment>
struct indexable<const Segment*>
{
    typedef const Segment* V;

    typedef const Segment& result_type;
    result_type operator()(const V& v) const { return *v; }
};

}}} // namespace boost::geometry::index

const void* RtreeVectorMap::cell(const Point& pos) const
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    double max_area_ratio = 0.;
    const void* cell_ptr = nullptr;

#if BOOST_VERSION >= 105500
    for (auto it = rtree_.qbegin(bgi::intersects(pos)), end = rtree_.qend(); it != end; ++it) {
        const cell_t& cell = *it;
#else
    rtree_.query(bgi::intersects(pos), boost::make_function_output_iterator([&](const cell_t& cell) {
#endif
        if (std::get<2>(cell) > max_area_ratio) {
            max_area_ratio = std::get<2>(cell);
            cell_ptr = &cell;
        }
#if BOOST_VERSION >= 105500
    }
#else
    }));
#endif

    return cell_ptr;
}

double RtreeVectorMap::minSquaredDistanceNearbyStructure(const Point& pos,
                                                         const void* cell_ptr,
                                                         double angle) const
{
    double min_squared_dist = std::numeric_limits<double>::max();

#if BOOST_VERSION >= 105600
    if (cell_ptr == nullptr)
        return min_squared_dist;

    const cell_t& cell = *static_cast<const cell_t*>(cell_ptr);

    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    for (auto it = std::get<1>(cell).qbegin(bgi::nearest(pos, 1)), end = std::get<1>(cell).qend(); it != end; ++it) {
        const Vector* const& segment = *it;
        min_squared_dist = bg::comparable_distance(pos, *segment);
    }
#endif

    return min_squared_dist;
}

double RtreeVectorMap::minDistanceNearbyStructure(const Point& pos) const
{
    return 0.;
}

double RtreeVectorMap::minSquaredDistanceNearbyStructure(const Point& pos) const
{
    return 0.;
}

bool RtreeVectorMap::structureNearby(const Point& pos,
                                     const double thresh) const
{
    return false;
}

bool RtreeVectorMap::retrieveFiltered(const Point& pos,
                                      Vectors& lines) const
{
    return false;
}

bool RtreeVectorMap::retrieve(const Point& pos,
                              Vectors& lines) const
{
    return false;
}

double RtreeVectorMap::intersectScanRay(const Vector& ray,
                                        const void* cell_ptr,
                                        double angle,
                                        double max_range) const
{
#if BOOST_VERSION >= 105600
    if (!cell_ptr)
        return max_range;

    const cell_t& cell = *static_cast<const cell_t*>(cell_ptr);

    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    // Boost.Geometry has a nice path tracing feature, but it's still experimental as of 24.04.2018
    const auto& celltree = std::get<1>(cell);
#ifdef BOOST_GEOMETRY_INDEX_DETAIL_EXPERIMENTAL
    double dist = max_range;
    for (auto it = celltree.qbegin(bgi::path(ray, 1)), end = celltree.qend(); it != end; ++it) {
        const Vector* const& segment = *it;
        std::vector<Point> tmp;
        bg::intersection(ray, *segment, tmp);
        dist = bg::distance(ray.first, tmp.front());
    }
    return dist;
#else
    // the workaround is to use intersects(), so we still have to find out the closest segment
    double min_squared_dist = std::numeric_limits<double>::max();
    std::vector<Point> tmp;
    for (auto it = celltree.qbegin(bgi::intersects(ray)), end = celltree.qend(); it != end; ++it) {
        const Vector* const& segment = *it;
        tmp.clear();
        bg::intersection(ray, *segment, tmp);
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

int RtreeVectorMap::intersectScanPattern(
    const Point& pos,
    const Vectors& pattern,
    IntersectionSet& intersections) const
{
    return 0;
}

void RtreeVectorMap::intersectScanPattern(const Point& pos,
                                          const Vectors& pattern,
                                          std::vector<float>& ranges,
                                          const float default_measurement) const
{

}

// preconditions: room_polygons.size() == room_segment_indices.size(), each room
// must be a non-intersecting clockwise ring, every value in
// room_segment_indices must be less than segments.size().
// The map is newly created using the given data.
void RtreeVectorMap::insert(const Vectors& segments,
                            const std::vector<polygon_t>& room_polygons,
                            const std::vector<std::vector<std::size_t>>& room_segment_indices)
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    data_ = segments;
    room_polygons_ = room_polygons;

    // We bulk-insert into the R-tree so that the efficient packing algorithm is
    // used. All values have to be passed to the constructor at once.
    std::vector<cell_t> values(room_polygons.size());
    std::size_t iroom = 0;
    for (const polygon_t& room_polygon : room_polygons_) {
        box_t envelope = bg::return_envelope<box_t>(room_polygon);
        std::vector<const Vector*> segment_pointers(room_segment_indices[iroom].size());
        std::size_t nsegment = 0;
        for (std::size_t segment_index : room_segment_indices[iroom])
            segment_pointers[nsegment++] = &data_[segment_index];
        values[iroom] = std::make_tuple(envelope, innertree_t(segment_pointers), bg::area(room_polygon) / bg::area(envelope));
        iroom++;
    }

    rtree_.~rtree();
    new (&rtree_) tree_t(values);
}

unsigned int RtreeVectorMap::handleInsertion()
{
    return 0;
}

void RtreeVectorMap::doLoad(const YAML::Node& node)
{
    VectorMap::doLoad(node);

    YAML::Binary room_sizes_binary = node["room_sizes"].as<YAML::Binary>();
    YAML::Binary room_indices_binary = node["room_indices"].as<YAML::Binary>();
    std::vector<std::uint32_t> room_sizes;
    std::vector<std::uint32_t> room_indices;
    serialization::deserialize(room_sizes_binary, room_sizes);
    serialization::deserialize(room_indices_binary, room_indices);

    std::vector<std::vector<std::size_t>> room_segment_indices(room_sizes.size());

    auto it1 = room_indices.begin();
    for(std::size_t i = 0; i < room_sizes.size(); ++i) {
        std::uint32_t room_size = room_sizes[i];
        std::vector<std::size_t>& segment_indices = room_segment_indices[i];
        segment_indices.resize(room_size);
        for(std::uint32_t j = 0; j < room_size; ++j, ++it1)
            segment_indices[j] = *it1;
    }

    YAML::Binary room_ring_sizes_binary = node["room_ring_sizes"].as<YAML::Binary>();
    YAML::Binary room_ring_data_binary = node["room_ring_data"].as<YAML::Binary>();
    std::vector<std::int32_t> room_ring_sizes;
    std::vector<Point> room_ring_data;
    serialization::deserialize(room_ring_sizes_binary, room_ring_sizes);
    serialization::deserialize(room_ring_data_binary, room_ring_data);

    std::vector<polygon_t> room_polygons;

    auto it2 = room_ring_data.begin();
    polygon_t* last_polygon = nullptr;
    for (std::size_t i = 0; i < room_ring_sizes.size(); ++i) {
        std::int32_t ring_size = room_ring_sizes[i];
        ring_t* ring;
        if (ring_size > 0) {
            room_polygons.push_back(polygon_t());
            last_polygon = &room_polygons.back();
            ring = &last_polygon->outer();
        } else {
            last_polygon->inners().push_back(ring_t());
            ring = &last_polygon->inners().back();
            ring_size = -ring_size;
        }
        ring->resize(ring_size);
        for (std::int32_t j = 0; j < ring_size; ++j, ++it2)
            (*ring)[j] = *it2;
    }

    insert(data_, room_polygons, room_segment_indices);
}

void RtreeVectorMap::doSave(YAML::Node& node) const
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    VectorMap::doSave(node);
    node["map_type"] = "rtree";

    std::vector<std::uint32_t> room_sizes;
    std::vector<std::uint32_t> room_indices;
    room_sizes.reserve(room_polygons_.size());
    room_indices.reserve(data_.size());

#if BOOST_VERSION >= 105900
    for (const cell_t& cell : rtree_) {
#else
    auto dummy_pred = [](const cell_t&) { return true; };
 #if BOOST_VERSION >= 105500
    for (auto it = rtree_.qbegin(bgi::satisfies(dummy_pred)), end = rtree_.qend(); it != end; ++it) {
        const cell_t& cell = *it;
 #else
    rtree_.query(bgi::satisfies(dummy_pred), boost::make_function_output_iterator([&](const cell_t& cell) {
 #endif
#endif
        room_sizes.push_back(std::get<1>(cell).size());
#if BOOST_VERSION >= 105900 || BOOST_VERSION < 105600
        for (const Vector* const& segment : std::get<1>(cell)) {
#else
        auto dummy_pred2 = [](const Vector* const&) { return true; };
        for (auto it2 = std::get<1>(cell).qbegin(bgi::satisfies(dummy_pred2)), end2 = std::get<1>(cell).qend(); it2 != end2; ++it2) {
            const Vector* const& segment = *it2;
#endif
            room_indices.push_back(static_cast<std::uint32_t>(segment - data_.data()));
        }
#if BOOST_VERSION >= 105500
    }
#else
    }));
#endif

    YAML::Binary room_sizes_binary;
    serialization::serialize(room_sizes, room_sizes_binary);
    node["room_sizes"] = room_sizes_binary;

    YAML::Binary room_indices_binary;
    serialization::serialize(room_indices, room_indices_binary);
    node["room_indices"] = room_indices_binary;

    std::vector<std::int32_t> room_ring_sizes;
    std::vector<Point> room_ring_data;

    for (const polygon_t& polygon : room_polygons_) {
        room_ring_sizes.push_back(polygon.outer().size());
        room_ring_data.insert(room_ring_data.end(), polygon.outer().begin(), polygon.outer().end());
        for (const ring_t& hole : polygon.inners()) {
            room_ring_sizes.push_back(-static_cast<std::int32_t>(hole.size()));
            room_ring_data.insert(room_ring_data.end(), hole.begin(), hole.end());
        }
    }

    YAML::Binary room_ring_sizes_binary;
    serialization::serialize(room_ring_sizes, room_ring_sizes_binary);
    node["room_ring_sizes"] = room_ring_sizes_binary;

    YAML::Binary room_ring_data_binary;
    serialization::serialize(room_ring_data, room_ring_data_binary);
    node["room_ring_data"] = room_ring_data_binary;
}

const RtreeVectorMap::tree_t& RtreeVectorMap::rtree() const
{
    return rtree_;
}

const std::vector<RtreeVectorMap::polygon_t>& RtreeVectorMap::room_polygons() const
{
    return room_polygons_;
}
