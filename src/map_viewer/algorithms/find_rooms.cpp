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

    // iterate through all the doors and try to find rooms!
    std::vector<std::vector<point_t>> rooms;
    for (const d::door_t& door : doors) {
        // first, find a face of the door that's not the door's right or left side
        d::node_t* door_nodes[] = {graph.corner_lookup[door[0].second]->node, graph.corner_lookup[door[1].second]->node};
        for (d::node_t* door_node : door_nodes) {
            d::node_t* last_node = door_node;
            d::node_t* current_node = nullptr; // initialized to suppress warning
            // find door segment first
            for (d::edge_t& door_edge : door_node->edges) {
                if (door_edge.door) {
                    current_node = door_edge.target->node;
                    std::cout << "start traversing from door edge "
                              << door_edge.start->point.x() << '|'
                              << door_edge.start->point.y() << "->"
                              << door_edge.target->point.x() << '|'
                              << door_edge.target->point.y() << '\n';
                    door_edge.checked = true;
                    break;
                }
            }
            std::vector<point_t> room;
            d::edge_t* edge = nullptr;
            // traverse room walls in clockwise order
            do {
                for (std::size_t i = 0, n = current_node->edges.size(); i < n; i++) {
                    if (current_node->edges[i].target->node == last_node) {
                        d::node_t* new_last_node = current_node;
                        d::edge_t* newedge = &current_node->edges[++i % n];
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
                room.push_back(edge->start->point);
            } while (current_node != door_node);
            room.push_back(edge->target->point);
            rooms.push_back(room);
next_door_face:
            ;
        }
    }
    std::cout << "Found " << rooms.size() << " rooms\n";
    return rooms;
}
