#include "rtree_vectormap_conversion.h"

#include "types.h"

#include <cslibs_vectormaps/maps/rtree_vector_map.h>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include <tuple>
#include <string>
#include <algorithm>

using namespace cslibs_vectormaps;

RtreeVectormapConversion::RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters) :
    parameters_(parameters)
{
}

void RtreeVectormapConversion::index_rooms(std::vector<std::vector<point_t>> rooms)
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    rooms_.clear();

    // We bulk-insert into R-tree, which uses more efficient packing algorithm.
    // All values have to be passed to the constructor at once.
    std::vector<std::tuple<box_t, box_t, std::size_t>> values(rooms.size());
    std::size_t nroom = 0;
    for (std::vector<point_t>& room : rooms) {
        ring_t ring(room.begin(), room.end());
        rooms_.push_back(ring);
        box_t envelope = bg::return_envelope<box_t>(ring);
        box_t envelope_with_leeway = envelope;
        point_t& min = envelope_with_leeway.min_corner();
        min.x(min.x() - parameters_.leeway);
        min.y(min.y() - parameters_.leeway);
        point_t& max = envelope_with_leeway.max_corner();
        max.x(max.x() + parameters_.leeway);
        max.y(max.y() + parameters_.leeway);
        values[nroom] = std::make_tuple(envelope_with_leeway, envelope, nroom);
        nroom++;
    }

    rtree_.~rtree();
    new (&rtree_) decltype(rtree_)(values);
}

void RtreeVectormapConversion::index_segments(std::vector<segment_t> segments)
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    segments_ = segments;
    segment_indices_.clear();
    segment_indices_.resize(rooms_.size());
    std::size_t i = 0;
    for (segment_t& segment : segments) {
        std::vector<std::tuple<box_t, box_t, std::size_t>> results;
        rtree_.query(bgi::intersects(segment), std::back_inserter(results));
        for (std::tuple<box_t, box_t, std::size_t>& result : results) {
            segment_indices_[std::get<2>(result)].push_back(i);
        }
        i++;
    }
}

// when dropping outliers, we allow for some leeway, so that we are not prone to rounding errors etc.
void RtreeVectormapConversion::drop_outliers()
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;

    // this commented line should work with Boost 1.59.0 and later (replacing the next 3 lines)
    //for (const std::tuple<box_t, box_t, std::size_t>& val : rtree_) {
    auto dummy_pred = [](const std::tuple<box_t, box_t, std::size_t>&){ return true; };
    for (auto it = rtree_.qbegin(bgi::satisfies(dummy_pred)), end = rtree_.qend(); it != end; ++it) {
        const std::tuple<box_t, box_t, std::size_t>& val = *it;
        std::vector<std::size_t> cleaned_indices;
        for (std::size_t segment_index : segment_indices_[std::get<2>(val)]) {
            if (bg::distance(segments_[segment_index], rooms_[std::get<2>(val)]) <= parameters_.leeway)
                cleaned_indices.push_back(segment_index);
        }
        segment_indices_[std::get<2>(val)].swap(cleaned_indices);
    }
}

bool RtreeVectormapConversion::save(point_t min, point_t max) const
{
    auto ends_with = [](const std::string& value, const std::string& ending) {
        return ending.size() <= value.size()
            && std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

    VectorMap::BoundingBox bounding(min, max);
    //VectorMap::Ptr map(new RtreeVectorMap(bounding, segments, rooms, segment_indices, true));

    const bool compress = ends_with(parameters_.path, std::string(".gzip"));
    //return map->save(parameters_.path, compress);
}
