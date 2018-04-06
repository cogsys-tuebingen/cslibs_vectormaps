#include "find_rooms.h"

#include <iostream>
#include <utility>

using namespace cslibs_vectormaps;

FindRooms::FindRooms(const FindRoomsParameter &parameter) : parameter_(parameter)
{
}

std::vector<std::vector<point_t>> FindRooms::find_rooms(const std::vector<FindDoors::door_t>& doors)
{
    typedef FindDoors d;
    d::graph_t& graph = *parameter_.graph;

    // remove eventual door edges from previous call
    for (d::node_t& node : graph.nodes) {
        for (auto it = node.edges.begin(), end = node.edges.end(); it != end;) {
            if (it->door) {
                it = node.edges.erase(it);
                end = node.edges.end();
            } else {
                ++it;
            }
        }
    }

    // add two additional edges per door that connect the door's sides
    for (const d::door_t& door : doors) {
        for (std::size_t side = 0; side < 2; side++) {
            d::corner_t* c1 = graph.corner_lookup[door[side].first];
            d::corner_t* c2 = graph.corner_lookup[door[(side + 1) % 2].second];
            d::edge_t e1 = {c2, c1, true, false};
            d::edge_t e2 = {c1, c2, true, false};
            c1->node->edges.push_back(e2);
            c2->node->edges.push_back(e1);
        }
    }

    // sort each node's edges in counter-clockwise order and remove duplicate edges again
    std::size_t erased2 = d::sort_edges_delete_duplicates(graph.nodes);
    std::cout << "Deleted " << erased2 << " duplicate edges\n";

    auto point_eq = [](const point_t& p1, const point_t& p2) {
        return p1.x() == p2.x() && p1.y() == p2.y();
    };

    // iterate through all the doors and try to find rooms!
    std::vector<std::vector<point_t>> rooms;
    for (const d::door_t& door : doors) {
        // first, find a face of the door that's not the door's right or left side
        d::node_t* door_nodes[] = {graph.corner_lookup[door[0].second]->node, graph.corner_lookup[door[1].second]->node};
        d::node_t* other_door_nodes[] = {graph.corner_lookup[door[1].first]->node, graph.corner_lookup[door[0].first]->node};
        for (d::node_t* door_node : door_nodes) {
            d::node_t* last_node = door_node;
            d::node_t* current_node = nullptr; // initialized to suppress warning
            d::edge_t* edge = nullptr;
            // find door segment first
            d::node_t* other_door_node = other_door_nodes[door_node == door_nodes[0] ? 0 : 1];
            for (d::edge_t& door_edge : door_node->edges) {
                if (door_edge.door && door_edge.target->node == other_door_node) {
                    current_node = door_edge.target->node;
                    std::cout << "start traversing from door edge "
                              << door_edge.start->point.x() << '|'
                              << door_edge.start->point.y() << "->"
                              << door_edge.target->point.x() << '|'
                              << door_edge.target->point.y() << '\n';
                    door_edge.checked = true;
                    edge = &door_edge;
                    break;
                }
            }
            std::vector<point_t> room = {edge->start->point};
            // traverse room walls in clockwise order
            do {
                auto add_point = [&point_eq](std::vector<point_t>& room, const point_t& p) {
                    if (!point_eq(p, room.back())) {
                        // add new point only if last point does not compare equal
                        if (room.size() <= 1 || !point_eq(p, *(room.end() - 2)))
                            // add new point only if it would not form a spike
                            room.push_back(p);
                        else
                            // otherwise, remove spike
                            room.pop_back();
                    }
                };
                for (std::size_t i = 0, n = current_node->edges.size(); i < n; i++) {
                    if (current_node->edges[i].target->node == last_node) {
                        d::node_t* new_last_node = current_node;
                        d::edge_t* new_edge = &current_node->edges[++i % n];
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
                            add_point(room, edge->target->point);
                        }
                        edge = new_edge;
                        if (edge->door) {
                            if (edge->checked) {
                                std::cout << "room might have already been found at door edge "
                                          << edge->start->point.x() << '|'
                                          << edge->start->point.y() << "->"
                                          << edge->target->point.x() << '|'
                                          << edge->target->point.y() << '\n';
                                goto next_door_face;
                            }
                            edge->checked = true;
                        }
                        current_node = edge->target->node;
                        last_node = new_last_node;
                        break;
                    }
                }
                add_point(room, edge->start->point);
            } while (current_node != door_node);
            room.push_back(edge->target->point);
            if (!point_eq(room.back(), room.front()))
                room.push_back(room.front());
            rooms.push_back(room);
next_door_face:
            ;
        }
    }
    std::cout << "Found " << rooms.size() << " rooms\n";
    return rooms;
}
