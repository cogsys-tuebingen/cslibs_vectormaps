#include "find_doors.h"

#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
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

}

FindDoors::FindDoors(const FindDoorsParameter& parameter) : parameter_(parameter)
{

}

void FindDoors::get_corners(const std::vector<segment_t>& segments, const FindDoorsParameter& params, std::vector<double>& positions, std::vector<std::vector<double>>& lines)
{
    std::map<double, std::set<double>> corner_map;

    for (const segment_t& s : segments) {
        if (params.find_mode == FindDoorsParameter::ROWS) {
            corner_map[s.first.y()].insert(s.first.x());
            corner_map[s.second.y()].insert(s.second.x());
        } else { // params.find_mode == FindDoorsParameter::COLUMNS
            corner_map[s.first.x()].insert(s.first.y());
            corner_map[s.second.x()].insert(s.second.y());
        }
    }

    positions.reserve(corner_map.size());
    lines.reserve(corner_map.size());

    for (const auto& pair : corner_map) {
        positions.emplace_back(pair.first);
        lines.emplace_back(pair.second.begin(), pair.second.end());
    }
}

std::size_t FindDoors::merge_nodes(const FindDoorsParameter& params, const std::vector<double>& cpositions, const std::vector<std::vector<double>>& clines, std::map<point_t, corner_t*, point_compare>& corner_lookup)
{
    double maxdist = params.merge_max_proximity;
    std::size_t merged = 0;
    auto mergenode = [&corner_lookup](point_t p1, point_t p2) {
        corner_t* c1 = corner_lookup[p1];
        corner_t* c2 = corner_lookup[p2];
        if (c1->node != c2->node) {
            std::vector<edge_t>& edges2 = c2->node->edges;
            for (const edge_t& edge : edges2) {
                edge.start->node = c1->node;
            }
            // c2->node from before the loop is still in the nodes vector, but not referenced by any corner anymore
            c1->node->edges.insert(c1->node->edges.end(), edges2.begin(), edges2.end());
            std::vector<edge_t>().swap(edges2); // clear edges of c2
        }
    };
    auto point = [&params](double a, double b) {
        return params.find_mode == FindDoorsParameter::ROWS ? point_t{a, b} : point_t{b, a};
    };
    std::size_t n = cpositions.size();
    for (std::size_t iy1 = 0; iy1 < n - 1; iy1++) {
        const std::vector<double>& line1 = clines[iy1];
        std::size_t nline1 = line1.size();
        for (std::size_t ix1 = 0; ix1 < nline1; ix1++) {
            double x1 = line1[ix1];
            double y1 = cpositions[iy1];
            if (ix1 > 0) {
                double x0 = line1[ix1 - 1];
                if (x1 - x0 <= maxdist) {
                    mergenode(point(x0, y1), point(x1, y1));
                    merged++;
                }
            }
            for (std::size_t iy2 = iy1 + 1; iy2 < n && cpositions[iy2] - y1 <= maxdist; iy2++) {
                double y2 = cpositions[iy2];
                for (double x2 : clines[iy2]) {
                    if (x1 - x2 <= maxdist && x2 - x1 <= maxdist) {
                        mergenode(point(x1, y1), point(x2, y2));
                        merged++;
                    } else if (x2 - x1 > maxdist) {
                        break;
                    }
                }
            }
        }
    }
    return merged;
}

std::size_t FindDoors::delete_inner_edges(std::vector<node_t>& nodes)
{
    std::size_t deleted = 0;
    for (node_t& node : nodes) {
        for (auto it = node.edges.begin(); it != node.edges.end();) {
            if (it->target->node == &node) {
                it = node.edges.erase(it);
                deleted++;
            } else {
                ++it;
            }
        }
    }
    return deleted;
}

std::size_t FindDoors::sort_edges_delete_duplicates(std::vector<node_t>& nodes)
{
    std::size_t erased = 0;
    for (node_t& node : nodes) {
        // because of the merging, there can be empty nodes that should not be considered
        if (node.edges.size() > 0) {
            // in order to sort the edges, we have to determine one of the node's points as a center point.
            const point_t& center = node.edges[0].start->point;
            // sort edges in anti-clockwise order.
            // see https://stackoverflow.com/a/6989383 for a bad explanation
            std::sort(node.edges.begin(), node.edges.end(), [&center](const edge_t& e1, const edge_t& e2) {
                const point_t& a = e1.target->point;
                const point_t& b = e2.target->point;
                if (a.x() - center.x() >= 0 && b.x() - center.x() < 0)
                    return false;
                if (a.x() - center.x() < 0 && b.x() - center.x() >= 0)
                    return true;
                if (a.x() - center.x() == 0 && b.x() - center.x() == 0) {
                    if (a.y() - center.y() >= 0 || b.y() - center.y() >= 0)
                        return a.y() < b.y();
                    return b.y() < a.y();
                }
                return (a.x() - center.x()) * (b.y() - center.y())
                        > (b.x() - center.x()) * (a.y() - center.y());
            });
            for (std::size_t s = node.edges.size(), i = 0; i < s && s > 1; i++) {
                std::size_t toerase = 0;
                for (std::size_t j = i; j < s - (i == 0)
                && node.edges[j].target->node == node.edges[(j + 1) % s].target->node/*
                && node.edges[j].target->point.x() == node.edges[(j + 1) % s].target->point.x()
                && node.edges[j].target->point.y() == node.edges[(j + 1) % s].target->point.y()*/;) {
                    toerase++;
                    j++;
                }
                auto begin = node.edges.begin();
                node.edges.erase(begin + i, begin + i + toerase);
                s -= toerase;
                erased += toerase;
            }
        }
    }
    return erased;
}

std::vector<segment_t> FindDoors::round_segments(const std::vector<segment_t>& segments) const
{
    double p = parameter_.map_precision;
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

std::vector<segment_t> FindDoors::clean_segments(const std::vector<segment_t>& rounded_segments) const
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

std::vector<FindDoors::door_t> FindDoors::find_doors(graph_t& graph, const std::vector<segment_t>& cleaned_segments)
{
    const FindDoorsParameter& params = parameter_;

    // get corners from line segments
    std::vector<double> cpositions;
    std::vector<std::vector<double>> clines;
    get_corners(cleaned_segments, params, cpositions, clines);

    // make vector of all corners and vector of all nodes
    graph.nodes.clear();
    graph.corners.clear();
    for (std::size_t i = 0, n = cpositions.size(); i < n; i++) {
        for (double p2 : clines[i]) {
            graph.nodes.push_back({});
            corner_t corner = {
                {
                    params.find_mode == FindDoorsParameter::ROWS ? p2 : cpositions[i],
                    params.find_mode == FindDoorsParameter::ROWS ? cpositions[i] : p2
                },
                nullptr // fill that later, because pointers to nodes are invalidated in this loop
            };
            graph.corners.push_back(corner);
        }
    }

    // make map for efficiently looking up corners by specifying coordinates
    graph.corner_lookup.clear();
    for (std::size_t i = 0, n = graph.corners.size(); i < n; i++) {
        graph.corners[i].node = &graph.nodes[i];
        graph.corner_lookup[graph.corners[i].point] = &graph.corners[i];
    }

    // then, connect those nodes. Each node's vector of connected nodes is populated.
    for (const segment_t& segment : cleaned_segments) {
        corner_t* c1 = graph.corner_lookup[segment.first];
        corner_t* c2 = graph.corner_lookup[segment.second];
        edge_t e1 = {c2, c1, false, false};
        edge_t e2 = {c1, c2, false, false};
        c1->node->edges.push_back(e2);
        c2->node->edges.push_back(e1);
    }

    // merge nodes that are close to each other
    std::size_t merged = merge_nodes(params, cpositions, clines, graph.corner_lookup);
    std::cout << "Merged " << merged << " nodes into neighboring nodes\n";

    // this can lead to redundant edges, remove those
    std::size_t deleted = delete_inner_edges(graph.nodes);
    std::cout << "Deleted " << deleted << " redundant edges connecting corners of the same node\n";

    // sort each node's edges in counter-clockwise order and remove duplicate edges (those can happen because of shitty map data)
    std::size_t erased1 = sort_edges_delete_duplicates(graph.nodes);
    std::cout << "Deleted " << erased1 << " duplicate edges\n";

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
    auto check_door_side = [&door_depth_min, &door_depth_max, &check_length, &check_angle](const point_t& v1, const point_t& v2, const point_t& v3) {
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
        return check_length(v2, door_depth_min, door_depth_max)
            /*&& check_angle(v1, v2) && check_angle(v3, v2)*/;
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
    };
    std::vector<const edge_t*> side_candidates;
    for (node_t& firstnode : graph.nodes) {
        // find all nodes that connect exactly two edges. those can be corner points of doors.
        if (firstnode.edges.size() != 2)
            continue;
        // if one of the connected nodes also has exactly two edges, the
        // three edges might form one of the two walls on either side of
        // the door
        if (check_node(firstnode, firstnode.edges[1], firstnode.edges[0]))
            side_candidates.push_back(&firstnode.edges[0]);
        if (check_node(firstnode, firstnode.edges[0], firstnode.edges[1]))
            side_candidates.push_back(&firstnode.edges[1]);
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
        const point_t& v21 = side_candidates[i1]->start->point;
        const point_t& v22 = side_candidates[i1]->target->point;
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
            const point_t& w21 = side_candidates[i2]->start->point;
            const point_t& w22 = side_candidates[i2]->target->point;
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
            const point_t& v21 = side_candidates[i1]->start->point;
            const point_t& v22 = side_candidates[i1]->target->point;
            const point_t& w21 = side_candidates[i2]->start->point;
            const point_t& w22 = side_candidates[i2]->target->point;
            door_t door_candidate = {segment_t{v21, v22}, segment_t{w21, w22}};
            door_candidates.push_back(door_candidate);
            door_candidate_distances.push_back(side_candidate_distances[i1]);
        }
    }
    std::cout << "Found " << door_candidates.size() << " door candidates\n";

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
            doors.push_back(door_candidate);
        }
    }
    std::cout << "Found " << doors.size() << " doors!\n";

    return doors;
}
