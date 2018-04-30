#include "find_doors.h"

#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/length.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/version.hpp>

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cstdint>

#include <algorithm>
#include <string>
#include <iostream>
#include <limits>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <array>
#include <iterator>

using namespace cslibs_vectormaps;

namespace {

// some utility boost::geometry is lacking
point_t add(const point_t& p1, const point_t& p2)
{
    point_t p = p1;
    boost::geometry::add_point(p, p2);
    return p;
}

point_t subtract(const point_t& p1, const point_t& p2)
{
    point_t p = p1;
    boost::geometry::subtract_point(p, p2);
    return p;
}

point_t multiply(const point_t& p, double v)
{
    point_t p2 = p;
    boost::geometry::multiply_value(p2, v);
    return p2;
}

point_t divide(const point_t& p, double v)
{
    point_t p2 = p;
    boost::geometry::divide_value(p2, v);
    return p2;
}

bool equal(const point_t& a, const point_t& b)
{
    return a.x() == b.x() && a.y() == b.y();
}

}

FindDoors::FindDoors(const FindDoorsParameter& parameter) : parameter_(parameter)
{

}

std::vector<segment_t> FindDoors::round_segments(const std::vector<segment_t>& segments, double precision)
{
    double p = precision;
    if (p != 0.) {
        std::vector<segment_t> rounded_segments = segments;
        for (segment_t& toround : rounded_segments) {
            toround.first.x(std::round(toround.first.x() * p) / p);
            toround.first.y(std::round(toround.first.y() * p) / p);
            toround.second.x(std::round(toround.second.x() * p) / p);
            toround.second.y(std::round(toround.second.y() * p) / p);
        }
        return rounded_segments;
    } else {
        return segments;
    }
}

std::vector<segment_t> FindDoors::clean_segments(const std::vector<segment_t>& rounded_segments)
{
    // make a new vector that only contains successive segments, no overlapping
    // segments. we can only replace exact vertical or exact horizontal
    // overlappings at the moment.
    std::vector<segment_t> non_overlapping_segments;

    // sort original segments into std::vectors (vertical, horizontal, other)
    std::vector<segment_t> vertical, horizontal;
    for (const segment_t& segment : rounded_segments) {
        if (segment.first.x() == segment.second.x()) {
            segment_t s = {
                {segment.first.x(), std::min(segment.first.y(), segment.second.y())},
                {segment.second.x(), std::max(segment.first.y(), segment.second.y())}
            };
            vertical.push_back(s);
        } else if (segment.first.y() == segment.second.y()) {
            segment_t s = {
                {std::min(segment.first.x(), segment.second.x()), segment.first.y()},
                {std::max(segment.first.x(), segment.second.x()), segment.second.y()}
            };
            horizontal.push_back(s);
        } else {
            non_overlapping_segments.push_back(segment);
        }
    }
    // sort the std::vectors
    std::sort(vertical.begin(), vertical.end(), [](const segment_t& s1, const segment_t& s2) {
        return s1.first.x() < s2.first.x()
               || s1.first.x() == s2.first.x()
                  && (s1.first.y() < s2.first.y()/*
                      || s1.first.y() == s2.first.y()
                         && s1.second.y() < s2.second.y()*/);
    });
    std::sort(horizontal.begin(), horizontal.end(), [](const segment_t& s1, const segment_t& s2) {
        return s1.first.y() < s2.first.y()
               || s1.first.y() == s2.first.y()
                  && (s1.first.x() < s2.first.x()/*
                      || s1.first.x() == s2.first.x()
                         && s1.second.x() < s2.second.x()*/);
    });
    // finally, detect overlapping segments and replace those by single segments
    std::size_t overlaps = 0;
    for (std::size_t i = 0, n = vertical.size(), j; i < n; i = j) {
        double miny = vertical[i].first.y();
        double maxy = vertical[i].second.y();
        for (j = i + 1; j < n && vertical[i].first.x() == vertical[j].first.x(); j++) {
            if (vertical[j].first.y() < maxy) {
                if (vertical[j].second.y() > maxy)
                    maxy = vertical[j].second.y();
                overlaps++;
            } else {
                break;
            }
        }
        segment_t s = {{vertical[i].first.x(), miny}, {vertical[i].first.x(), maxy}};
        non_overlapping_segments.push_back(s);
    }
    for (std::size_t i = 0, n = horizontal.size(), j; i < n; i = j) {
        double minx = horizontal[i].first.x();
        double maxx = horizontal[i].second.x();
        for (j = i + 1; j < n && horizontal[i].first.y() == horizontal[j].first.y(); j++) {
            if (horizontal[j].first.x() < maxx) {
                if (horizontal[j].second.x() > maxx)
                    maxx = horizontal[j].second.x();
                overlaps++;
            } else {
                break;
            }
        }
        segment_t s = {{minx, horizontal[i].first.y()}, {maxx, horizontal[i].first.y()}};
        non_overlapping_segments.push_back(s);
    }
    std::cout << "Merged " << overlaps << " overlapping segments\n";

    // TODO: put the following lines in another function
    // TODO: workaround for older Boost using bounding boxes
#if BOOST_VERSION < 105600
    std::cerr << "Boost version >= 1.56.0 is required for creating the segment graph! The results are probably unusable.\n";
    return non_overlapping_segments;
#else
    // replace intersecting line segments by multiple new segments that end in a common intersection point
    // use a spatial index because finding intersecting line segments globally is hard
    namespace bgi = boost::geometry::index;
    bgi::rtree<segment_t, bgi::rstar<16>> rtree(non_overlapping_segments);
    std::vector<segment_t> clean_segments;
    for (const segment_t& query_segment : non_overlapping_segments) {
        std::vector<point_t> intersections;
        for (auto qit = rtree.qbegin(bgi::intersects(query_segment)), end = rtree.qend(); qit != end; ++qit) {
            const segment_t& result_segment = *qit;
            boost::geometry::intersection(result_segment, query_segment, intersections);
        }
        std::set<point_t, point_compare> newpoints(intersections.begin(), intersections.end());
        for (auto pit1 = newpoints.begin(), pit2 = std::next(pit1), end = newpoints.end(); pit2 != end; ++pit1, ++pit2) {
            clean_segments.push_back({*pit1, *pit2});
        }
    }
    std::cout << "Created " << (clean_segments.size() - non_overlapping_segments.size()) << " new segments because of intersections\n";

    return clean_segments;
#endif
}

std::vector<FindDoors::door_t> FindDoors::find_doors(const std::vector<segment_t>& cleaned_segments)
{
    const FindDoorsParameter& params = parameter_;

    // find doors
    double door_depth_min = params.door_depth_min;
    double door_depth_max = params.door_depth_max;
    double door_width_min = params.door_width_min;
    double door_width_max = params.door_width_max;

    auto check_length = [](const point_t& v, double min, double max) {
        // checks the length of a vector
        double d = v.x() * v.x() + v.y() * v.y();
        return d >= min * min && d <= max * max;
    };
    auto check_angle = [&params](const point_t& v1, const point_t& v2) {
        double angle = std::atan2(v2.y(), v2.x()) - std::atan2(v1.y(), v1.x());
        if (angle < 0)
            angle += boost::math::double_constants::two_pi;
        return angle >= boost::math::double_constants::half_pi - params.door_angle_diff_max
            && angle <= boost::math::double_constants::half_pi + params.door_angle_diff_max;
    };
    /*auto check_door_side = [&door_depth_min, &door_depth_max, &check_length, &check_angle](const point_t& v1, const point_t& v2, const point_t& v3) {
        // Checks if v1, v2, v3 could form the side of a door like this:
        //     v1
        //  +<----+
        //        |v2
        //        v
        //  +<----+
        //     v3
        // This could be the left side of a door. The length of v2 is
        // checked. angle(v1, v2) and angle(v3, v2) are checked to be
        // approximately 90 deg. 270 deg does not count! This is so that
        // v2's direction is consistent and so that no duplicates occur
        // when considering edges in both directions.
        // The angles are not checked anymore right now.
        return check_length(v2, door_depth_min, door_depth_max);
            //&& check_angle(v1, v2) && check_angle(v3, v2);
    };
    auto check_node = [&check_door_side](const node_t& firstnode, const edge_t& e1, const edge_t& e2) {
        // checks if edge e2 from firstnode to secondnode could form
        // the side of a door as explained in check_door_side() above
        const node_t* secondnode = e2.target->node;
        if (secondnode->edges.size() == 2) {
            const edge_t& e3 = secondnode->edges[
                secondnode->edges[0].target->node == &firstnode ? 1 : 0
            ];
            auto edge_to_vector = [](const edge_t& e) {
                return subtract(e.target->point, e.start->point);
            };
            if (check_door_side(edge_to_vector(e1), edge_to_vector(e2), edge_to_vector(e3)))
                return true;
        }
        return false;
    };*/
    std::vector<segment_t> side_candidates;
    /*for (const node_t& firstnode : graph.nodes) {
        // find all nodes that connect exactly two edges. those can be corner points of doors.
        if (firstnode.edges.size() != 2)
            continue;
        // if one of the connected nodes also has exactly two edges, the
        // three edges might form one of the two walls on either side of
        // the door
        if (check_node(firstnode, firstnode.edges[1], firstnode.edges[0]))
            side_candidates.emplace_back(firstnode.edges[0].start->point, firstnode.edges[0].target->point);
        if (check_node(firstnode, firstnode.edges[0], firstnode.edges[1]))
            side_candidates.emplace_back(firstnode.edges[1].start->point, firstnode.edges[1].target->point);
    }*/
    // here, segments become door side candidates if their length fits.
    // just forget the madness above.
    for (const segment_t& segment : cleaned_segments) {
        if (check_length(subtract(segment.second, segment.first), door_depth_min, door_depth_max)) {
            side_candidates.push_back(segment);
            side_candidates.emplace_back(segment.second, segment.first);
        }
    }
    std::cout << "Found " << side_candidates.size() << " door side candidates\n";
    
    std::size_t ncandidates = side_candidates.size();
    std::vector<std::size_t> side_candidate_partners(ncandidates, static_cast<std::size_t>(-1));
    std::vector<double> side_candidate_distances(ncandidates, std::numeric_limits<double>::max());
    for (std::size_t i1 = 0; i1 < ncandidates; i1++) {
        // we search for the other side of the door frame by drawing
        // a ray r from the middle of the edge like this:
        //     v1
        //  +<----+          +---->+
        //        |  v       ^
        //      v2|----->----|--- r
        //        v          |w2
        //  +<----+          +---->+
        //     v3
        // If r cuts another possible door frame side nearly
        // perpendicularly and that intersection point is not too far
        // and not too close from v2, we found a door.
        const point_t& v21 = side_candidates[i1].first;
        const point_t& v22 = side_candidates[i1].second;
        // r = middle + t * v
        point_t middle = multiply(add(v21, v22), .5);
        point_t v2 = subtract(v22, v21);
        point_t v = {-v2.y(), v2.x()}; // 90 deg turn
        double v_length = std::sqrt(v.x() * v.x() + v.y() * v.y());

        // check all other candidates
        for (std::size_t i2 = 0; i2 < ncandidates; i2++) {
            if (i2 == i1)
                continue;
            // line intersection adapted from https://stackoverflow.com/a/565282/1007605
            auto cross = [](const point_t& v, const point_t& w) {
                return v.x() * w.y() - v.y() * w.x();
            };
            const point_t& w21 = side_candidates[i2].first;
            const point_t& w22 = side_candidates[i2].second;
            point_t w2 = subtract(w22, w21);
            double det = cross(v, w2);
            // check if vectors are neither parallel nor collinear
            if (det == 0.)
                continue;
            // check that second side is cut by ray
            double u = cross(subtract(w21, middle), v) / det;
            if (u < 0. || u > 1.)
                continue;
            // check distance to second side
            double d = cross(subtract(w21, middle), w2) * v_length / det;
            if (d < door_width_min || d > door_width_max)
                continue;
            // check that sides are approximately parallel
            if (!check_angle(v, w2))
                continue;
            // we found a match! if there are multiple matches we only keep the closest
            if (d < side_candidate_distances[i1]) {
                side_candidate_distances[i1] = d;
                side_candidate_partners[i1] = i2;
            }
            if (d < side_candidate_distances[i2]) {
                side_candidate_distances[i2] = d;
                side_candidate_partners[i2] = i1;
            }
        }
    }

    // match side candidate pairs
    std::vector<door_t> door_candidates;
    std::vector<double> door_candidate_distances;
    for (std::size_t i1 = 0; i1 < ncandidates; i1++) {
        std::size_t i2 = side_candidate_partners[i1];
        if (i2 != static_cast<std::size_t>(-1) && i1 < i2
        && side_candidate_partners[i2] == i1) {
            const point_t& v21 = side_candidates[i1].first;
            const point_t& v22 = side_candidates[i1].second;
            const point_t& w21 = side_candidates[i2].first;
            const point_t& w22 = side_candidates[i2].second;
            door_t door_candidate = {segment_t{v21, v22}, segment_t{w21, w22}};
            door_candidates.push_back(door_candidate);
            door_candidate_distances.push_back(side_candidate_distances[i1]);
        }
    }
    std::cout << "Found " << door_candidates.size() << " door candidates\n";

#if BOOST_VERSION < 105600
    std::cerr << "Boost version >= 1.56.0 is required for checking space between door frames, so you might get more false positives\n";
    return door_candidates;
#else
    // check if area between door frame sides is unobstructed
    std::vector<door_t> doors;
    namespace bgi = boost::geometry::index;
    bgi::rtree<segment_t, bgi::rstar<16>> rtree(cleaned_segments);
    for (std::size_t i = 0; i < door_candidates.size(); i++) {
        const door_t& door_candidate = door_candidates[i];
        double d1 = boost::geometry::distance(door_candidate[0].first, door_candidate[0].second);
        double d2 = boost::geometry::distance(door_candidate[1].first, door_candidate[1].second);

        // aligned at the middle of the longer door frame side, create a
        // rectangle between the door sides. it looks like this:
        //      door_width
        //      +---------+
        //
        //        +-----+             +
        //  ----+ |     | +----       |
        // frame| |rect | |frame      | door_width * unobstructed_area_depth
        //  ----+ |     | +----       |
        //        +-----+             +
        //
        //        +-----+
        // door_width * unobstructed_area_width
        const segment_t& longer_side = door_candidate[d1 > d2 ? 0 : 1];
        double longer_side_length = d1 > d2 ? d1 : d2;
        const point_t& v21 = longer_side.first;
        const point_t& v22 = longer_side.second;
        // r = middle + t * v
        point_t middle = multiply(add(v21, v22), .5);
        point_t v2 = subtract(v22, v21);
        point_t v = {-v2.y(), v2.x()}; // 90 deg turn
        double factor = door_candidate_distances[i] / longer_side_length;
        // t2 and t are as long as the door is wide, t2 points in direction of
        // the first door side, t points perpendicularly to the other door side
        point_t t2 = multiply(v2, factor);
        point_t t = multiply(v, factor);
        double rectangle_length = params.unobstructed_area_depth;
        double width_padding = .5 - params.unobstructed_area_width * .5;
        point_t p1 = add(add(middle, multiply(t,      width_padding)), multiply(t2, -.5 * rectangle_length));
        point_t p2 = add(add(middle, multiply(t, 1. - width_padding)), multiply(t2, -.5 * rectangle_length));
        point_t p3 = add(add(middle, multiply(t, 1. - width_padding)), multiply(t2,  .5 * rectangle_length));
        point_t p4 = add(add(middle, multiply(t,      width_padding)), multiply(t2,  .5 * rectangle_length));

        // if this rectangle is not obstructed, we found an actual door
        ring_t unobstructed_area;
        unobstructed_area.insert(unobstructed_area.end(), {p1, p2, p3, p4, p1});
        std::vector<segment_t> result;
        bgi::query(rtree, bgi::intersects(unobstructed_area), std::back_inserter(result));
        if (result.size() == 0) {
            door_t d = {
                segment_t{door_candidate[0].second, door_candidate[1].first},
                segment_t{door_candidate[1].second, door_candidate[0].first}
            };
            doors.push_back(d);
        }
    }

    // near the door frames, the door might still intersect the map. prune door sides appropriately
    for (door_t& door : doors) {
        for (segment_t& side : door) {
            std::vector<segment_t> intersections;
            bgi::query(rtree, bgi::intersects(side), std::back_inserter(intersections));
            for (const segment_t& intersection : intersections) {
                // work around buggy boost::geometry::intersection rarely giving incorrect result
                if (equal(intersection.first, side.first) || equal(intersection.first, side.second)
                || equal(intersection.second, side.first) || equal(intersection.second, side.second))
                    continue;
                std::vector<point_t> intersection_points;
                boost::geometry::intersection(side, intersection, intersection_points);
                for (const point_t& intersection_point : intersection_points) {
                    segment_t newside1 = {side.first, intersection_point};
                    segment_t newside2 = {intersection_point, side.second};
                    side = boost::geometry::length(newside1) > boost::geometry::length(newside2)
                         ? newside1 : newside2;
                }
            }
        }
    }
    std::cout << "Found " << doors.size() << " doors!\n";
    return doors;
#endif
}
