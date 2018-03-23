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
    std::size_t iroom = 0;
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
        values[iroom] = std::make_tuple(envelope_with_leeway, envelope, iroom);
        iroom++;
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

    // In order to drop non-referenced segments entirely, we first build
    // std::vectors of actual segments for each room and also build a single
    // std::set (to delete duplicates), transform this set into a single
    // std::vector, create a std::map that maps all the segments in this
    // std::vector to their indices and finally create a new std::vector per
    // room that contains only indices into the new big std::vector.

    struct segment_compare {
        // somewhere in the STL, there must be a shortcut for this...
        bool operator()(const segment_t& l, const segment_t& r) const
        {
            return l.first.x() < r.first.x()
                   || l.first.x() == r.first.x()
                      && (l.first.y() < r.first.y()
                          || l.first.y() == r.first.y()
                             && (l.second.x() < r.second.x()
                                 || l.second.x() == r.second.x()
                                    && l.second.y() < r.second.y()));
        }
    };

    std::size_t original_size = segments_.size();
    std::vector<std::vector<segment_t>> rooms_segments(rtree_.size());
    std::set<segment_t, segment_compare> segment_set;

    std::size_t iroom = 0;
    // the next line should work with Boost >= 1.59.0 (replacing the 3 lines that follow)
    //for (const std::tuple<box_t, box_t, std::size_t>& val : rtree_) {
    auto dummy_pred = [](const std::tuple<box_t, box_t, std::size_t>&){ return true; };
    for (auto it = rtree_.qbegin(bgi::satisfies(dummy_pred)), end = rtree_.qend(); it != end; ++it) {
        const std::tuple<box_t, box_t, std::size_t>& val = *it;
        std::vector<segment_t>& room_segments = rooms_segments[iroom++];
        for (std::size_t segment_index : segment_indices_[std::get<2>(val)]) {
            if (bg::distance(segments_[segment_index], rooms_[std::get<2>(val)]) <= parameters_.leeway) {
                room_segments.push_back(segments_[segment_index]);
                segment_set.insert(segments_[segment_index]);
            }
        }
    }

    segments_.assign(segment_set.begin(), segment_set.end());
    std::map<segment_t, std::size_t, segment_compare> segment_index_map;
    for (std::size_t i = 0, s = segments_.size(); i < s; i++) {
        segment_index_map[segments_[i]] = i;
    }

    for (std::size_t i = 0, s = segment_indices_.size(); i < s; i++) {
        std::vector<std::size_t> room_segment_indices(rooms_segments[i].size());
        for (std::size_t j = 0, t = rooms_segments[i].size(); j < t; j++)
            room_segment_indices[j] = segment_index_map[rooms_segments[i][j]];
        segment_indices_[i].swap(room_segment_indices);
    }

    std::cout << "Only " << segments_.size() << " out of " << original_size << " segments were exported as part of the selected rooms.\n";
}

bool RtreeVectormapConversion::save(point_t min, point_t max) const
{
    auto ends_with = [](const std::string& value, const std::string& ending) {
        return ending.size() <= value.size()
            && std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

    VectorMap::BoundingBox bounding(min, max);
    RtreeVectorMap::Ptr map(new RtreeVectorMap(bounding, true));

    map->insert(segments_, rooms_, segment_indices_);
    const bool compress = ends_with(parameters_.path, std::string(".gzip"));
    return map->save(parameters_.path, compress);
}
