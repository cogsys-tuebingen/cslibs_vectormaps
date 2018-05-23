#include "find_rooms.h"

#include <boost/version.hpp>
#if BOOST_VERSION >= 105600
#if BOOST_VERSION < 106000
#include <iostream>
#endif
// this goof forgot to include its iostream dependency until 1.60
#include <boost/geometry/algorithms/is_valid.hpp>
#endif
// this gimp does not seem to work without additional stuff
#include <boost/geometry/algorithms/within.hpp>
// so let's include everything
#include <boost/geometry.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <boost/function_output_iterator.hpp>

#include <iostream>
#include <utility>
#include <algorithm>

using namespace cslibs_vectormaps;

FindRooms::FindRooms(const FindRoomsParameter &parameter) : parameter_(parameter)
{
}

void FindRooms::get_corners(std::vector<double>& cpositions, std::vector<std::vector<double>>& clines, const std::vector<segment_t>& segments, const std::vector<FindDoors::door_t>& doors) const
{
    const FindRoomsParameter& params = parameter_;
    std::map<double, std::set<double>> corner_map;

    auto add_segment = [&params, &corner_map](const segment_t& s) {
        if (params.find_mode == FindRoomsParameter::ROWS) {
            corner_map[s.first.y()].insert(s.first.x());
            corner_map[s.second.y()].insert(s.second.x());
        } else { // params.find_mode == FindRoomsParameter::COLUMNS
            corner_map[s.first.x()].insert(s.first.y());
            corner_map[s.second.x()].insert(s.second.y());
        }
    };

    for (const segment_t& s : segments) {
        add_segment(s);
    }

    for (const FindDoors::door_t& door : doors) {
        for (const segment_t& s : door) {
            add_segment(s);
        }
    }

    cpositions.reserve(corner_map.size());
    clines.reserve(corner_map.size());

    for (const auto& pair : corner_map) {
        cpositions.emplace_back(pair.first);
        clines.emplace_back(pair.second.begin(), pair.second.end());
    }
}

std::size_t FindRooms::merge_nodes(std::map<point_t, corner_t*, point_compare>& corner_lookup, const std::vector<double>& cpositions, const std::vector<std::vector<double>>& clines) const
{
    const FindRoomsParameter& params = parameter_;
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
        return params.find_mode == FindRoomsParameter::ROWS ? point_t{a, b} : point_t{b, a};
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

std::size_t FindRooms::delete_inner_edges(graph_t& graph) const
{
    std::size_t deleted = 0;
    for (node_t& node : graph.nodes) {
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

std::size_t FindRooms::sort_edges_delete_duplicates(graph_t& graph) const
{
    std::size_t erased = 0;
    for (node_t& node : graph.nodes) {
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

std::size_t FindRooms::connect_lone_corners(graph_t& graph) const
{
    const FindRoomsParameter& params = parameter_;
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;
    struct point_getter {
        using result_type = const point_t&;
        result_type& operator()(corner_t* const& corner) const
        {
            return corner->point;
        }
    };
    std::vector<corner_t*> lone_corners;
    for (corner_t& corner : graph.corners) {
        if (corner.node->edges.size() == 1)
            lone_corners.push_back(&corner);
    }
    bgi::rtree<corner_t*, bgi::rstar<16>, point_getter> rtree(lone_corners);
    std::size_t connected = 0;

    // check neighborhood of each lone node
    double connect_max_proximity_squared = params.connect_max_proximity * params.connect_max_proximity;
    for (corner_t*& lone_corner : lone_corners) {
        // found lone node
        corner_t* candidate = lone_corner->node->edges[0].start;
        corner_t* partner = nullptr;
        bool found_self = false;
        double dist_squared_partner;
        // find queried node and the 2 closest other nodes.
        auto pred = bgi::nearest(candidate->point, 3);
        rtree.query(pred, boost::make_function_output_iterator([&](corner_t* const& corner) {
            double dist_squared = bg::comparable_distance(corner->point, candidate->point);
            if (dist_squared == 0.) {
                found_self = true;
            } else if (partner == nullptr) {
                dist_squared_partner = dist_squared;
                partner = corner;
            } else {
                corner_t* next_corner = corner;
                if (dist_squared < dist_squared_partner) {
                    std::swap(partner, next_corner);
                    std::swap(dist_squared, dist_squared_partner);
                }
                if (found_self
                && dist_squared_partner <= connect_max_proximity_squared
                && dist_squared > connect_max_proximity_squared
                && partner->node->edges.size() == 1) {
                    // found exactly one other lone node within threshold! connect them.
                    edge_t e1 = {candidate, partner, false, false};
                    edge_t e2 = {partner, candidate, false, false};
                    lone_corner->node->edges.push_back(e1);
                    partner->node->edges.push_back(e2);
                    connected++;
                }
            }
        }));
    }

    return connected;
}

void FindRooms::create_graph(graph_t& graph, const std::vector<segment_t>& cleaned_segments, const std::vector<FindDoors::door_t>& doors) const
{
    const FindRoomsParameter& params = parameter_;

    // get corners from line segments
    std::vector<double> cpositions;
    std::vector<std::vector<double>> clines;
    get_corners(cpositions, clines, cleaned_segments, doors);

    // make vector of all corners and vector of all nodes
    graph.nodes.clear();
    graph.corners.clear();
    for (std::size_t i = 0, n = cpositions.size(); i < n; i++) {
        for (double p2 : clines[i]) {
            graph.nodes.push_back({});
            corner_t corner = {
                {
                    params.find_mode == FindRoomsParameter::ROWS ? p2 : cpositions[i],
                    params.find_mode == FindRoomsParameter::ROWS ? cpositions[i] : p2
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
    for (const FindDoors::door_t& door : doors) {
        for (std::size_t side = 0; side < 2; side++) {
            corner_t* c1 = graph.corner_lookup[door[side].first];
            corner_t* c2 = graph.corner_lookup[door[side].second];
            edge_t e1 = {c2, c1, true, false};
            edge_t e2 = {c1, c2, true, false};
            c1->node->edges.push_back(e2);
            c2->node->edges.push_back(e1);
        }
    }

    // merge nodes that are close to each other
    std::size_t merged = merge_nodes(graph.corner_lookup, cpositions, clines);
    std::cout << "Merged " << merged << " nodes into neighboring nodes\n";

    // this can lead to redundant edges, remove those
    std::size_t deleted = delete_inner_edges(graph);
    std::cout << "Deleted " << deleted << " redundant edges connecting corners of the same node\n";

    // sort each node's edges in counter-clockwise order and remove duplicate edges (those can happen because of shitty map data)
    std::size_t erased1 = sort_edges_delete_duplicates(graph);
    std::cout << "Deleted " << erased1 << " duplicate edges\n";

    // close gaps between lone endpoints by inserting additional edges
    std::size_t connected = connect_lone_corners(graph);
    std::cout << "Closed " << connected << " gaps between lone endpoints\n";

    // we have to sort again...
    std::size_t erased2 = sort_edges_delete_duplicates(graph);
    std::cout << "Deleted " << erased2 << " duplicate edges\n";
}

std::vector<polygon_t> FindRooms::find_rooms(const std::vector<segment_t>& cleaned_segments, const std::vector<FindDoors::door_t>& doors) const
{
    graph_t graph;
    create_graph(graph, cleaned_segments, doors);

    // remove eventual door edges from previous call
    /*for (node_t& node : graph.nodes) {
        for (auto it = node.edges.begin(), end = node.edges.end(); it != end;) {
            if (it->door) {
                it = node.edges.erase(it);
                end = node.edges.end();
            } else {
                ++it;
            }
        }
    }*/

    auto point_eq = [](const point_t& p1, const point_t& p2) {
        return p1.x() == p2.x() && p1.y() == p2.y();
    };

    // iterate through all the doors and try to find rooms!
    std::vector<ring_t> rings;
    for (const FindDoors::door_t& door : doors) {
        // first, find a face of the door that's not the door's right or left side
        node_t* door_nodes[] = {graph.corner_lookup[door[0].first]->node, graph.corner_lookup[door[1].first]->node};
        node_t* other_door_nodes[] = {graph.corner_lookup[door[0].second]->node, graph.corner_lookup[door[1].second]->node};
        for (node_t* door_node : door_nodes) {
            node_t* last_node = door_node;
            node_t* current_node = nullptr; // initialized to suppress warning
            edge_t* edge = nullptr;
            // find door segment first
            node_t* other_door_node = other_door_nodes[door_node == door_nodes[0] ? 0 : 1];
            for (edge_t& door_edge : door_node->edges) {
                if (door_edge.door && door_edge.target->node == other_door_node) {
                    current_node = door_edge.target->node;
                    /*std::cout << "traverse from door side "
                              << door_edge.start->point.x() << '|'
                              << door_edge.start->point.y() << "->"
                              << door_edge.target->point.x() << '|'
                              << door_edge.target->point.y() << '\n';*/
                    door_edge.checked = true;
                    edge = &door_edge;
                    break;
                }
            }
            ring_t ring;
            ring.push_back(edge->start->point);
            // traverse room walls in clockwise order
            do {
                auto add_point = [&point_eq](ring_t& ring, const point_t& p) {
                    if (!point_eq(p, ring.back())) {
                        // add new point only if last point does not compare equal
                        if (ring.size() <= 1 || !point_eq(p, *(ring.end() - 2)))
                            // add new point only if it would not form a spike
                            ring.push_back(p);
                        else
                            // otherwise, remove spike
                            ring.pop_back();
                    }
                };
                for (std::size_t i = 0, n = current_node->edges.size(); i < n; i++) {
                    if (current_node->edges[i].target->node == last_node) {
                        node_t* new_last_node = current_node;
                        edge_t* new_edge = &current_node->edges[++i % n];
                        // edge->target->node == current_node can happen when one node comprises multiple corners
                        std::size_t j;
                        for (j = 0; new_edge->target->node == current_node && j < n - 1; j++) {
                            new_edge = &current_node->edges[++i % n];
                        }
                        if (j == n - 1) {
                            // dead end - we have to go back and find another junction
                            // just swap current and last node to revert direction
                            std::swap(last_node, current_node);
                            break;
                        }
                        if (edge != nullptr) {
                            add_point(ring, edge->target->point);
                        }
                        edge = new_edge;
                        if (edge->door) {
                            if (edge->checked) {
                                /*std::cout << "room might have already been found at door edge "
                                          << edge->start->point.x() << '|'
                                          << edge->start->point.y() << "->"
                                          << edge->target->point.x() << '|'
                                          << edge->target->point.y() << '\n';*/
                                goto next_door_face;
                            }
                            edge->checked = true;
                        }
                        current_node = edge->target->node;
                        last_node = new_last_node;
                        break;
                    }
                }
                add_point(ring, edge->start->point);
            } while (current_node != door_node);
            ring.push_back(edge->target->point);
            if (!point_eq(ring.back(), ring.front()))
                ring.push_back(ring.front());
            rings.push_back(ring);
next_door_face:
            ;
        }
    }

#if BOOST_VERSION < 105600
    std::cerr << "Boost version >= 1.56.0 is required for finding holes in rooms, so your results might be flawed\n";
#endif

    // look for holes and drop invalid rings, form polygons
    std::vector<polygon_t> rooms;
    std::vector<ring_t> possible_holes;
    std::size_t invalid = 0;
    for (ring_t& ring : rings) {
#if BOOST_VERSION >= 105600
        boost::geometry::validity_failure_type failure;
        if (boost::geometry::is_valid(ring, failure)) {
#endif
            rooms.push_back(polygon_t());
            rooms.back().outer() = ring;
#if BOOST_VERSION >= 105600
        } else if (failure == boost::geometry::failure_wrong_orientation) {
            possible_holes.push_back(ring);
        } else {
            std::string f;
            boost::geometry::is_valid(ring, f);
            std::cout << f << '\n';
            invalid++;
        }
#endif
    }

    // insert holes into polygons
    std::size_t holes = 0;
#if BOOST_VERSION >= 105600
    for (const ring_t& possible_hole : possible_holes) {
        for (polygon_t& room : rooms) {
            if (boost::geometry::within(possible_hole, room)) {
                room.inners().push_back(possible_hole);
                holes++;
                goto next_possible_hole;
            }
        }
        invalid++;
next_possible_hole:
        ;
    }
#endif

    std::cout << "Dropped " << invalid << " invalid rooms\n";
    std::cout << "Found " << holes << " holes in rooms\n";
    std::cout << "Found " << rooms.size() << " rooms!\n";
    return rooms;
}
