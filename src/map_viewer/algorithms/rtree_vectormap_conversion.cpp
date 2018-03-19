#include "rtree_vectormap_conversion.h"

#include <cslibs_vectormaps/maps/rtree_vector_map.h>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/math/constants/constants.hpp>

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <cstdint>

#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <array>
#include <random>
#include <iterator>

using namespace cslibs_vectormaps;

RtreeVectormapConversion::RtreeVectormapConversion(const RtreeVectormapConversionParameter& parameters) :
    parameters_(parameters)
{
}

bool RtreeVectormapConversion::operator()(const std::vector<QLineF>& vectors,
                                          const QPointF& min,
                                          const QPointF& max,
                                          progress_t progress)
{
    auto convertPoint = [](const QPointF &p) {
        return VectorMap::Point(p.x(), p.y());
    };
    auto ends_with = [](const std::string& value, const std::string& ending) {
        return ending.size() <= value.size()
            && std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    };

    VectorMap::BoundingBox bounding(convertPoint(min), convertPoint(max));
    VectorMap::Ptr map(new RtreeVectorMap(bounding, true));

    VectorMap::Vectors converted_vectors;
    for(std::size_t i = 0, s = vectors.size(); i < s; ++i) {
        progress(static_cast<int>(i / s));
        const QLineF& v = vectors[i];
        converted_vectors.emplace_back(convertPoint(v.p1()), convertPoint(v.p2()));
    }
    progress(-1);
    std::vector<std::vector<point_t>> rooms = find_rooms(converted_vectors, parameters_);
    //dynamic_cast<RtreeVectorMap&>(*map).create(converted_vectors, parameters_);
    const bool compress = ends_with(parameters_.path, std::string(".gzip"));
    return map->save(parameters_.path, compress);
}

namespace {

struct node_t;

using point_t = boost::geometry::model::d2::point_xy<double>;
using segment_t = boost::geometry::model::segment<point_t>;

struct corner_t {
    point_t point;
    node_t* node;
};

struct edge_t {
    corner_t* start;
    corner_t* target;
    bool door, checked;
};

struct node_t {
    std::vector<edge_t> edges;
};

// for sorting points
struct point_compare {
    bool operator()(const point_t& l, const point_t& r) const
    {
        return l.x() < r.x() || l.x() == r.x() && l.y() < r.y();
    }
};

}

// some other utility boost::geometry is lacking
static inline point_t operator+(const point_t& p1, const point_t& p2)
{
    point_t p = p1;
    boost::geometry::add_point(p, p2);
    return p;
}

static inline point_t operator-(const point_t& p1, const point_t& p2)
{
    point_t p = p1;
    boost::geometry::subtract_point(p, p2);
    return p;
}

static inline point_t operator*(const point_t& p, double v)
{
    point_t p2 = p;
    boost::geometry::multiply_value(p2, v);
    return p2;
}

static inline point_t operator/(const point_t& p, double v)
{
    point_t p2 = p;
    boost::geometry::divide_value(p2, v);
    return p2;
}

static void get_corners(const std::vector<segment_t>& segments, const RtreeVectormapConversionParameter& params, std::vector<double>& positions, std::vector<std::vector<double>>& lines)
{
    std::map<double, std::set<double>> corner_map;

    for (const segment_t& s : segments) {
        if (params.find_door_mode == RtreeVectormapConversionParameter::ROWS) {
            corner_map[s.first.y()].insert(s.first.x());
            corner_map[s.second.y()].insert(s.second.x());
        } else { // params.find_door_mode == RtreeVectormapConversionParameter::COLUMNS
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

static std::size_t merge_nodes(const RtreeVectormapConversionParameter& params, const std::vector<double>& cpositions, const std::vector<std::vector<double>>& clines, std::map<point_t, corner_t*, point_compare>& corner_lookup)
{
    double maxdist = params.merge_max_proximity * params.map_precision;
    std::size_t merged = 0;
    auto mergenode = [&corner_lookup](point_t p1, point_t p2) {
        corner_t* c1 = corner_lookup[p1];
        corner_t* c2 = corner_lookup[p2];
        if (c1->node != c2->node) {
            std::vector<edge_t>& edges2 = c2->node->edges;
            c1->node->edges.insert(c1->node->edges.end(), edges2.begin(), edges2.end());
            std::vector<edge_t>().swap(edges2); // clear edges of c2
            c2->node = c1->node; // the previous node is still in the nodes vector, it just is not referenced by any corner anymore
        }
    };
    auto point = [&params](double a, double b) {
        return params.find_door_mode == RtreeVectormapConversionParameter::ROWS ? point_t{a, b} : point_t{b, a};
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

static std::size_t sort_edges_delete_duplicates(std::vector<node_t>& nodes)
{
    std::size_t erased = 0;
    for (node_t& node : nodes) {
        // because of the merging, there can be empty nodes that should not be considered
        if (node.edges.size() > 0) {
            // in order to sort the edges, we have to determine one of the node's points as a center point.
            const point_t& center = node.edges[0].start->point;
            std::sort(node.edges.begin(), node.edges.end(), [&center](const edge_t& e1, const edge_t& e2) {
                return (e1.target->point.x() - center.x()) * (e2.target->point.y() - center.y())
                        > (e2.target->point.x() - center.x()) * (e1.target->point.y() - center.y());
            });
            for (std::size_t s = node.edges.size(), i = 0; i < s && s > 1; i++) {
                std::size_t toerase = 0;
                for (std::size_t j = i; j < s - (i == 0)
                && node.edges[j].target->point.x() == node.edges[(j + 1) % s].target->point.x()
                && node.edges[j].target->point.y() == node.edges[(j + 1) % s].target->point.y();) {
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

std::vector<std::vector<point_t>> RtreeVectormapConversion::find_rooms(const std::vector<segment_t>& segments, const RtreeVectormapConversionParameter& params)
{
    std::vector<segment_t> roundedsegments;
    for (const segment_t& segment : segments) {
        segment_t roundedsegment = segment;
        if (params.map_precision) {
            double p = params.map_precision;
            roundedsegment.first.x(std::round(roundedsegment.first.x() * p));
            roundedsegment.first.y(std::round(roundedsegment.first.y() * p));
            roundedsegment.second.x(std::round(roundedsegment.second.x() * p));
            roundedsegment.second.y(std::round(roundedsegment.second.y() * p));
        }
        roundedsegments.push_back(roundedsegment);
    }

    // first of all, make a new vector that only contains successive
    // segments, no overlapping segments. we can only replace exact vertical
    // or exact horizontal overlappings at the moment.
    std::vector<segment_t> clean_segments;
    {
        // sort original segments into std::vectors (vertical, horizontal, other)
        std::vector<segment_t> vertical, horizontal;
        for (const segment_t& segment : roundedsegments) {
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
                clean_segments.push_back(segment);
            }
        }
        // sort the std::vectors
        std::sort(vertical.begin(), vertical.end(), [](const segment_t& s1, const segment_t& s2) {
            return s1.first.x() < s2.first.x() || s1.first.x() == s2.first.x() && (s1.first.y() < s2.first.y()/* || s1.first.y() == s2.first.y() && s1.second.y() < s2.second.y()*/);
        });
        std::sort(horizontal.begin(), horizontal.end(), [](const segment_t& s1, const segment_t& s2) {
            return s1.first.y() < s2.first.y() || s1.first.y() == s2.first.y() && (s1.first.x() < s2.first.x()/* || s1.first.x() == s2.first.x() && s1.second.x() < s2.second.x()*/);
        });
        // finally, detect overlapping segments and replace those
        std::size_t overlaps = 0;
        for (std::size_t i = 0, n = vertical.size(), j; i < n; i = j) {
            double maxy = vertical[i].second.y();
            std::set<double> ys = {vertical[i].first.y(), vertical[i].second.y()};
            for (j = i + 1; j < n && vertical[i].first.x() == vertical[j].first.x(); j++) {
                if (vertical[j].first.y() < maxy) {
                    ys.insert(vertical[j].first.y());
                    ys.insert(vertical[j].second.y());
                    if (vertical[j].second.y() > maxy)
                        maxy = vertical[j].second.y();
                } else {
                    break;
                }
            }
            bool begin = true;
            double y_last;
            for (double y : ys) {
                if (begin) {
                    begin = false;
                } else {
                    segment_t s = {{vertical[i].first.x(), y_last}, {vertical[i].first.x(), y}};
                    clean_segments.push_back(s);
                }
                y_last = y;
            }
            if (i + 1 < j) {
                overlaps++;
            }
        }
        for (std::size_t i = 0, n = horizontal.size(), j; i < n; i = j) {
            double maxx = horizontal[i].second.x();
            std::set<double> xs = {horizontal[i].first.x(), horizontal[i].second.x()};
            for (j = i + 1; j < n && horizontal[i].first.y() == horizontal[j].first.y(); j++) {
                if (horizontal[j].first.x() < maxx) {
                    xs.insert(horizontal[j].first.x());
                    xs.insert(horizontal[j].second.x());
                    if (horizontal[j].second.x() > maxx)
                        maxx = horizontal[j].second.x();
                } else {
                    break;
                }
            }
            bool begin = true;
            double x_last;
            for (double x : xs) {
                if (begin) {
                    begin = false;
                } else {
                    segment_t s = {{x_last, horizontal[i].first.y()}, {x, horizontal[i].first.y()}};
                    clean_segments.push_back(s);
                }
                x_last = x;
            }
            if (i + 1 < j) {
                overlaps++;
            }
        }
        std::cout << "Eliminated " << overlaps << " overlaps\n";
    }

    // get corners from line segments
    std::vector<double> cpositions;
    std::vector<std::vector<double>> clines;
    get_corners(clean_segments, params, cpositions, clines);

    // make vector of all corners and vector of all nodes
    std::vector<node_t> nodes;
    std::vector<corner_t> corners;
    for (std::size_t i = 0, n = cpositions.size(); i < n; i++) {
        for (double p2 : clines[i]) {
            nodes.push_back({});
            corner_t corner = {
                {
                    params.find_door_mode == RtreeVectormapConversionParameter::ROWS ? p2 : cpositions[i],
                    params.find_door_mode == RtreeVectormapConversionParameter::ROWS ? cpositions[i] : p2
                },
                nullptr // fill that later, because pointers to nodes are invalidated in this loop
            };
            corners.push_back(corner);
        }
    }

    // make map for efficiently looking up corners by specifying coordinates
    std::map<point_t, corner_t*, point_compare> corner_lookup;
    for (std::size_t i = 0, n = corners.size(); i < n; i++) {
        corners[i].node = &nodes[i];
        corner_lookup[corners[i].point] = &corners[i];
    }

    // then, connect those nodes. Each node's vector of connected nodes is populated.
    for (const segment_t& segment : clean_segments) {
        corner_t* c1 = corner_lookup[segment.first];
        corner_t* c2 = corner_lookup[segment.second];
        edge_t e1 = {c2, c1, false, false};
        edge_t e2 = {c1, c2, false, false};
        c1->node->edges.push_back(e2);
        c2->node->edges.push_back(e1);
    }

    // merge nodes that are close to each other
    std::size_t merged = merge_nodes(params, cpositions, clines, corner_lookup);
    std::cout << "Merged " << merged << " nodes into neighboring nodes\n";

    // sort each node's edges in counter-clockwise order and remove duplicate edges (those can happen because of shitty map data)
    std::size_t erased1 = sort_edges_delete_duplicates(nodes);
    std::cout << "Deleted " << erased1 << " duplicate edges\n";

    // find doors
    std::vector<std::array<const edge_t, 2>> doors;
    {
        double door_depth_min = params.door_depth_min;
        double door_depth_max = params.door_depth_max;
        double door_width_min = params.door_width_min;
        double door_width_max = params.door_width_max;
        if (params.map_precision) {
            double p = params.map_precision;
            door_depth_min = std::round(door_depth_min * p);
            door_depth_max = std::round(door_depth_max * p);
            door_width_min = std::round(door_width_min * p);
            door_width_max = std::round(door_width_max * p);
        }

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
            return check_length(v2, door_depth_min, door_depth_max)
                && check_angle(v1, v2) && check_angle(v3, v2);
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
                    return e.target->point - e.start->point;
                };
                if (check_door_side(edge_to_vector(e1), edge_to_vector(e2), edge_to_vector(e3)))
                    return true;
            }
            return false;
        };
        std::list<const edge_t*> door_side_candidates;
        for (node_t& firstnode : nodes) {
            // find all nodes that connect exactly two edges. those can be corner points of doors.
            if (firstnode.edges.size() != 2)
                continue;
            // if one of the connected nodes also has exactly two edges, the
            // three edges might form one of the two walls on either side of
            // the door
            if (check_node(firstnode, firstnode.edges[1], firstnode.edges[0]))
                door_side_candidates.push_back(&firstnode.edges[0]);
            if (check_node(firstnode, firstnode.edges[0], firstnode.edges[1]))
                door_side_candidates.push_back(&firstnode.edges[1]);
        }
        std::cout << "Found " << door_side_candidates.size() << " door side candidates\n";

        for (auto it1 = door_side_candidates.begin(), end = door_side_candidates.end(); it1 != door_side_candidates.end();) {
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
            const point_t& v21 = (*it1)->start->point;
            const point_t& v22 = (*it1)->target->point;
            // r = middle + t * v
            point_t middle = (v21 + v22) * .5;
            point_t v2 = v22 - v21;
            point_t v = {-v2.y(), v2.x()}; // 90 deg turn
            double v_length = std::sqrt(v.x() * v.x() + v.y() * v.y());

            // check all other candidates
            for (auto it2 = door_side_candidates.begin(); it2 != end; ++it2) {
                if (it2 == it1)
                    continue;
                // line intersection adapted from https://stackoverflow.com/a/565282/1007605
                auto cross = [](const point_t& v, const point_t& w) {
                    return v.x() * w.y() - v.y() * w.x();
                };
                const point_t& w21 = (*it2)->start->point;
                const point_t& w22 = (*it2)->target->point;
                point_t w2 = w22 - w21;
                double det = cross(v, w2);
                // check if vectors are neither parallel nor collinear
                if (det == 0.)
                    continue;
                // check that second side is cut by ray
                double u = cross(w21 - middle, v) / det;
                if (u < 0. || u > 1.)
                    continue;
                // check distance to second side
                double d = cross(w21 - middle, w2) * v_length / det;
                if (d < door_width_min || d > door_width_max)
                    continue;
                // check that sides are approximately parallel
                if (!check_angle(v, w2))
                    continue;
                // we found a door! add it and remove candidates
                doors.push_back({**it1, **it2});
                door_side_candidates.erase(it2);
                it1 = door_side_candidates.erase(it1);
                goto next_door_side_candidate;
            }
            ++it1;
next_door_side_candidate:
            ;
        }
        std::cout << "Found " << doors.size() << " doors!\n";
    }

    // add two additional connections per door that connect the door's sides
    for (const std::array<const edge_t, 2>& door : doors) {
        for (std::size_t side = 0; side < 2; side++)  {
            corner_t* c1 = door[side].start;
            corner_t* c2 = door[(side + 1) % 2].target;
            edge_t e1 = {c2, c1, true, false};
            edge_t e2 = {c1, c2, true, false};
            c1->node->edges.push_back(e2);
            c2->node->edges.push_back(e1);
        }
    }

    // sort each node's edges in counter-clockwise order and remove duplicate edges again
    std::size_t erased2 = sort_edges_delete_duplicates(nodes);
    std::cout << "Deleted " << erased2 << " duplicate edges\n";

    // iterate through all the doors and try to find rooms!
    std::vector<std::vector<point_t>> rooms;
    for (const std::array<const edge_t, 2>& door : doors) {
        // first, find a face of the door that's not the door's right or left side
        node_t* door_nodes[] = {door[0].target->node, door[1].target->node};
        for (node_t* door_node : door_nodes) {
            node_t* last_node = door_node;
            node_t* current_node = nullptr; // initialized to suppress warning
            // find door segment first
            for (const edge_t& door_edge : door_node->edges) {
                if (door_edge.door) {
                    current_node = door_edge.target->node;
                    std::cout << "start traversing from door edge " << door_edge.start->point.x() << '|' << door_edge.start->point.y() << "->" << door_edge.target->point.x() << '|' << door_edge.target->point.y() << '\n';
                    break;
                }
            }
            std::vector<point_t> room;
            edge_t* edge = nullptr;
            // traverse room walls in clockwise order
            do {
                for (std::size_t i = 0, n = current_node->edges.size(); i < n; i++) {
                    if (current_node->edges[i].target->node == last_node) {
                        node_t* new_last_node = current_node;
                        edge_t* newedge = &current_node->edges[++i % n];
                        // edge->target->node == current_node can happen when one node comprises multiple corners
                        std::size_t j;
                        for (j = 0; newedge->target->node == current_node && j < n - 1; j++) {
                            newedge = &current_node->edges[++i % n];
                        }
                        if (j == n - 1) {
                            // dead end - we have to go back and find another junction
                            // just swap current and last node to revert direction
                            std::swap(last_node, current_node);
                            break;
                        }
                        if (edge != nullptr && edge->target != newedge->start) {
                            room.push_back(edge->target->point);
                        }
                        edge = newedge;
                        if (edge->door) {
                            if (edge->checked) {
                                std::cout << "room might have already been found at door edge " << edge->start->point.x() << '|' << edge->start->point.y() << "->" << edge->target->point.x() << '|' << edge->target->point.y() << '\n';
                                goto next_door_face;
                            }
                            edge->checked = true;
                        }
                        current_node = edge->target->node;
                        last_node = new_last_node;
                        break;
                    }
                }
                room.push_back(edge->start->point);
            } while (current_node != door_node);
            room.push_back(edge->target->point);
            if (room.size() < 900) // TODO: remove that shit, find the corridors for real
            rooms.push_back(room);
next_door_face:
            ;
        }
    }
    std::cout << "Found " << rooms.size() << " rooms\n";
    return rooms;
}
