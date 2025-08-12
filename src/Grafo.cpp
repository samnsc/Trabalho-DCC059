#include "Grafo.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "No.h"

// a delegating constructor is used here because const class members have to be
// instantiated before the initializer
Grafo::Grafo(bool is_directed, bool has_weighted_edges, bool has_weighted_nodes)
    : IN_DIRECIONADO{is_directed},
      IN_PONDERADO_ARESTA{has_weighted_edges},
      IN_PONDERADO_VERTICE{has_weighted_nodes} {}

void Grafo::createNode(char id, int weight) {
    if (this->lista_adj.find(id) == lista_adj.end()) {
        this->lista_adj[id] = std::unique_ptr<No>{new No{id, weight}};
    } else {
        throw std::runtime_error("Tried creating node on an already used ID.\n");
    }

    this->ordem++;
}

void Grafo::createEdge(char starting_id, char destination_id, int weight) {
    if (this->lista_adj.find(starting_id) != lista_adj.end() && this->lista_adj.find(destination_id) != lista_adj.end()) {
        this->lista_adj[starting_id]->createEdge(destination_id, weight);
    } else {
        throw std::runtime_error("Tried creating edge for inexistent nodes.\n");
    }

    if (!this->IN_DIRECIONADO) {
        // no check is done here for if the nodes exist since if that check were to be
        // equal to false the program would have stopped before reaching here
        this->lista_adj[destination_id]->createEdge(starting_id, weight);
    }
}

const std::map<char, std::unique_ptr<No>> &Grafo::getListaDeAdjacencia() const {
    return this->lista_adj;
}

int Grafo::getOrdem() const {
    return this->ordem;
}

void Grafo::printGraph() const {
    std::cout << (this->IN_DIRECIONADO ? "1" : "0") << " "
              << (this->IN_PONDERADO_ARESTA ? "1" : "0") << " "
              << (this->IN_PONDERADO_VERTICE ? "1" : "0") << "\n";

    std::cout << this->ordem << "\n";

    for (const auto &node : this->lista_adj) {
        std::cout << node.first;

        if (this->IN_PONDERADO_VERTICE) {
            std::cout << " " << node.second->getPeso();
        }

        std::cout << "\n";
    }

    std::map<char, std::vector<char>> printed_edges;
    for (const auto &node : this->lista_adj) {
        auto &current_node_printed_list = printed_edges[node.first];

        for (const auto &edge : node.second->getArestas()) {
            auto destination_position_in_printed_edges = std::find(current_node_printed_list.begin(), current_node_printed_list.end(), edge->getIdNoAlvo());

            // this initially checks if the graph is directed, if it is there are no duplicate edges to be removed, so the second condition isn't checked
            // if it's not directed it then checks if the destination node doesn't exist in the already printed edges vector to prevent duplicate edges from being printed (i.e. (a, b) and (b, a))
            if (this->IN_DIRECIONADO || destination_position_in_printed_edges == current_node_printed_list.end()) {
                std::cout << node.first << " " << edge->getIdNoAlvo();

                if (this->IN_PONDERADO_ARESTA) {
                    std::cout << " " << edge->getPeso();
                }

                std::cout << "\n";

                printed_edges[edge->getIdNoAlvo()].push_back(node.first);
            } else {
                // this instance is erased because there can be multiple edges that have the same starting and destination nodes,
                // if it wasn't erased they would be collpased into a single edge when printing
                current_node_printed_list.erase(destination_position_in_printed_edges);
            }
        }
    }
}

void Grafo::printAdjacencyList() const {
    std::string formatted_adjacency_list = this->formattedAdjacencyList();

    std::cout << formatted_adjacency_list;
}

std::string Grafo::formattedAdjacencyList() const {
    std::stringstream formatted_adjacency_list;

    for (const auto &node : this->lista_adj) {
        formatted_adjacency_list << node.first << ": ";

        const auto &edges = node.second->getArestas();

        if (!edges.empty()) {
            formatted_adjacency_list << edges[0]->getIdNoAlvo();
        }

        for (int i = 1; i < edges.size(); i++) {
            formatted_adjacency_list << " -> " << edges[i]->getIdNoAlvo();
        }

        formatted_adjacency_list << "\n";
    }

    return formatted_adjacency_list.str();
}

std::vector<char> Grafo::fechoTransitivoDireto(char id_no) const {
    if (this->lista_adj.find(id_no) == this->lista_adj.end()) {
        throw std::runtime_error("Tried getting the direct transitive closure for an inexistent node.\n");
    }

    std::vector<char> direct_transitive_closure;
    const auto &node = this->lista_adj.at(id_no);

    this->directTransitiveClosureHelper(node, direct_transitive_closure);

    return direct_transitive_closure;
}

void Grafo::directTransitiveClosureHelper(const std::unique_ptr<No> &node, std::vector<char> &direct_transitive_closure) const {
    for (const auto &edge : node->getArestas()) {
        // to prevent an infinite loop we check if the current node is already part of the closure, if it is we won't search it since it was already searched
        if (std::find(direct_transitive_closure.begin(), direct_transitive_closure.end(), edge->getIdNoAlvo()) == direct_transitive_closure.end()) {
            direct_transitive_closure.push_back(edge->getIdNoAlvo());
            this->directTransitiveClosureHelper(this->lista_adj.at(edge->getIdNoAlvo()), direct_transitive_closure);
        }
    }
}

std::vector<char> Grafo::fechoTransitivoIndireto(char id_no) const {
    if (this->lista_adj.find(id_no) == this->lista_adj.end()) {
        throw std::runtime_error("Tried getting the indirect transitive closure for an inexistent node.\n");
    }

    std::vector<char> indirect_transitive_closure;
    std::vector<char> cant_reach_target_node;

    for (const auto &node : this->lista_adj) {
        std::vector<char> scoured_nodes;

        if (
            std::find(indirect_transitive_closure.begin(), indirect_transitive_closure.end(), node.first) != indirect_transitive_closure.end() ||  // checks if the current node has already been found to be able to reach the target node
            std::find(cant_reach_target_node.begin(), cant_reach_target_node.end(), node.first) != cant_reach_target_node.end()                    // checks if the current node has already been found to *NOT* be able to reach the target node
        ) {
            continue;
        }

        bool reachable = indirectTransitiveClosureHelper(id_no, node.second, indirect_transitive_closure, cant_reach_target_node, scoured_nodes);

        if (reachable) {
            indirect_transitive_closure.insert(indirect_transitive_closure.end(), scoured_nodes.cbegin(), scoured_nodes.cend());
        } else {
            cant_reach_target_node.insert(cant_reach_target_node.end(), scoured_nodes.cbegin(), scoured_nodes.cend());
        }
    }

    return indirect_transitive_closure;
}

bool Grafo::indirectTransitiveClosureHelper(char target_node_id, const std::unique_ptr<No> &current_node, const std::vector<char> &indirect_transitive_closure, const std::vector<char> &cant_reach_target_node, std::vector<char> &scoured_nodes) const {
    scoured_nodes.push_back(current_node->getId());

    for (const auto &edge : current_node->getArestas()) {
        if (
            edge->getIdNoAlvo() == target_node_id ||
            std::find(indirect_transitive_closure.begin(), indirect_transitive_closure.end(), edge->getIdNoAlvo()) != indirect_transitive_closure.end()  // checks if the simbling node has already been found to be able to reach the target node
        ) {
            return true;
        }

        if (
            std::find(scoured_nodes.begin(), scoured_nodes.end(), edge->getIdNoAlvo()) != scoured_nodes.end() ||                          // checks if the sibling node has already been searched
            std::find(cant_reach_target_node.begin(), cant_reach_target_node.end(), edge->getIdNoAlvo()) != cant_reach_target_node.end()  // checks if the simbling node has already been found to *NOT* be able to reach the target node
        ) {
            continue;
        }

        bool reachable = this->indirectTransitiveClosureHelper(target_node_id, this->lista_adj.at(edge->getIdNoAlvo()), indirect_transitive_closure, cant_reach_target_node, scoured_nodes);

        if (reachable) {
            return true;
        }
    }

    return false;
}

std::vector<char> Grafo::caminhoMinimoDijkstra(char id_no_a, char id_no_b) const {
    if (!this->IN_PONDERADO_ARESTA) {
        throw std::runtime_error("Tried finding the shortest path on an edge-unweighted graph.\n");
    }

    if (
        this->lista_adj.find(id_no_a) == this->lista_adj.end() ||
        this->lista_adj.find(id_no_b) == this->lista_adj.end()
    ) {
        throw std::runtime_error("Tried finding the shortest path for an inexistent node.\n");
    }

    // stores tuples in the following format:
    // node id, current distance, has been travelled to, current path
    std::vector<std::tuple<char, int, bool, std::vector<char>>> distance_to_node;

    for (const auto &node : this->lista_adj) {
        distance_to_node.push_back({node.first, std::numeric_limits<int>::max(), false, {}});  // since the distance is being stored as an int there is no concept of infinity, so the maximum number an int can store is used instead
    }

    // set the starting node distance to 0
    auto starting_node_iterator = std::find_if(  // searches for the iterator pointing to the starting node
        distance_to_node.begin(),
        distance_to_node.end(),
        [&id_no_a](const std::tuple<char, int, bool, std::vector<char>> &tuple) { return std::get<0>(tuple) == id_no_a; }
    );
    std::get<1>(*starting_node_iterator) = 0;
    std::get<2>(*starting_node_iterator) = true;
    std::get<3>(*starting_node_iterator) = {id_no_a};

    this->dijkstraShortestPathHelper(*starting_node_iterator, 0, distance_to_node);

    auto destination_node_iterator = std::find_if(  // searches for the iterator pointing to the starting node
        distance_to_node.begin(),
        distance_to_node.end(),
        [&id_no_b](const std::tuple<char, int, bool, std::vector<char>> &tuple) { return std::get<0>(tuple) == id_no_b; }
    );
    if (destination_node_iterator != distance_to_node.end()) {  // if there are no possible paths between the two given nodes an empty vector is returned
        return std::get<3>(*destination_node_iterator);
    } else {
        return {};
    }
}

void Grafo::dijkstraShortestPathHelper(const std::tuple<char, int, bool, std::vector<char>> &current_node, int summed_distance, std::vector<std::tuple<char, int, bool, std::vector<char>>> &distance_to_node) const {
    // setting the distances relative to the current node
    for (const auto &edge : this->lista_adj.at(std::get<0>(current_node))->getArestas()) {
        auto iterator = std::find_if(  // searches for the iterator pointing to the node the edge points to
            distance_to_node.begin(),
            distance_to_node.end(),
            [&edge](const std::tuple<char, int, bool, std::vector<char>> &tuple) { return std::get<0>(tuple) == edge->getIdNoAlvo(); }
        );

        if ((edge->getPeso() + summed_distance) < std::get<1>(*iterator)) {  // will only change the distance value if the sum is smaller than the already stored distance
            std::get<1>(*iterator) = edge->getPeso() + summed_distance;
            std::get<2>(*iterator) = false;  // sets the travelled indicator to false, if it wasn't already set to that, signaling that it can go through that node again

            std::get<3>(*iterator).assign(std::get<3>(current_node).begin(), std::get<3>(current_node).end());
            std::get<3>(*iterator).push_back(std::get<0>(*iterator));
        }
    }

    // sorts the distance_to_node vector after changing the distances
    std::sort(
        distance_to_node.begin(),
        distance_to_node.end(),
        [](const std::tuple<char, int, bool, std::vector<char>> &first, const std::tuple<char, int, bool, std::vector<char>> &second) {
            return std::get<1>(first) < std::get<1>(second);
        }
    );

    for (auto &next_node : distance_to_node) {
        if (!std::get<2>(next_node)) {  // checks if the node has already been travelled to
            // checks if the node's distance is smaller than the int maximum (in this case representing infinity), if it isn't,
            // that means that this graph isn't connected and it has reached all nodes that the starting node is connected to
            if (std::get<1>(next_node) >= std::numeric_limits<int>::max()) {
                break;
            }

            std::get<2>(next_node) = true;
            this->dijkstraShortestPathHelper(next_node, std::get<1>(next_node), distance_to_node);
            break;
        }
    }
}

std::vector<char> Grafo::caminhoMinimoFloyd(char id_no_a, char id_no_b) const {
    auto distances = this->floydAllDistances();

    if (distances[id_no_a].find(id_no_b) != distances[id_no_a].end()) {
        return distances[id_no_a][id_no_b].second;
    } else {
        return {};
    }
}

std::map<char, std::map<char, std::pair<int, std::vector<char>>>> Grafo::floydAllDistances() const {
    std::map<char, std::map<char, std::pair<int, std::vector<char>>>> distances;
    std::vector<char> all_node_ids;

    for (const auto &node : this->lista_adj) {
        all_node_ids.push_back(node.first);
        distances[node.first][node.first] = {0, {node.first}};  // sets the distance to itself to 0

        for (const auto &edge : node.second->getArestas()) {
            distances[node.first][edge->getIdNoAlvo()] = {edge->getPeso(), {node.first, edge->getIdNoAlvo()}};  // sets the distance to all of its direct neighbors
        }
    }

    for (auto passing_through_node : all_node_ids) {
        for (auto starting_node : all_node_ids) {
            for (auto ending_node : all_node_ids) {
                int current_distance;
                if (distances[starting_node].find(ending_node) == distances[starting_node].end()) {  // checks if there is a path between both nodes
                    current_distance = std::numeric_limits<int>::max();
                } else {
                    current_distance = distances[starting_node][ending_node].first;
                }

                int new_distance;
                std::vector<char> new_path;
                // checks if there is a path passing that goes from the starting node to the ending node while passing through the new node
                // if there isn't there is no need to compare if the distances are bigger or not so we can just continue onto the next node
                if (
                    distances[starting_node].find(passing_through_node) == distances[starting_node].end() ||
                    distances[passing_through_node].find(ending_node) == distances[passing_through_node].end()
                ) {
                    continue;
                } else {
                    new_distance = distances[starting_node][passing_through_node].first + distances[passing_through_node][ending_node].first;

                    // sets the new path by summing both paths together
                    new_path.assign(distances[starting_node][passing_through_node].second.begin(), distances[starting_node][passing_through_node].second.end());
                    // the start iterator is moved up one space to prevent the passing_through_node id from being duplicated, since it will always end one path and start the other
                    new_path.insert(new_path.end(), (distances[passing_through_node][ending_node].second.begin() + 1), distances[passing_through_node][ending_node].second.end());
                }

                if (new_distance < current_distance) {
                    distances[starting_node][ending_node] = {new_distance, new_path};
                }
            }
        }
    }

    return distances;
}

std::unique_ptr<Grafo> Grafo::arvoreGeradoraMinimaPrim(std::vector<char> ids_nos) const {
    if (!this->IN_PONDERADO_ARESTA || this->IN_DIRECIONADO) {
        throw std::runtime_error("Tried generating a minimum spanning tree on an edge-unweighted or directed graph.\n");
    }

    if (ids_nos.empty()) {
        return std::unique_ptr<Grafo>{new Grafo{this->IN_DIRECIONADO, this->IN_PONDERADO_ARESTA, this->IN_PONDERADO_VERTICE}};
    }

    // this checks if all of the nodes selected for the minimum spanning tree are connected, if they aren't its not possible to make a
    // minimum spanning tree out of these nodes, since a minimum spanning tree can only be made on an undirected graph we only need
    // to check if node is connected to the rest, if it is that means all of the other ones also are, if it isn't that means that these
    // nodes can't form a connected graph
    auto direct_transitive_closure = this->fechoTransitivoDireto(ids_nos[0]);
    for (auto node_id : ids_nos) {
        if (std::find(direct_transitive_closure.begin(), direct_transitive_closure.end(), node_id) == direct_transitive_closure.end()) {
            throw std::runtime_error("Tried generating a minimum spanning tree while not all nodes on the graph are connected.\n");
        }
    }

    auto edges = this->getEdges(ids_nos);

    std::sort(  // sorts the list of edges
        edges.begin(),
        edges.end(),
        [](const std::tuple<char, int, char, int, int> &first, const std::tuple<char, int, char, int, int> &second) {  // sorts the vector by each edge's weight
            return std::get<4>(first) < std::get<4>(second);
        }
    );

    auto graph = std::unique_ptr<Grafo>{new Grafo{this->IN_DIRECIONADO, this->IN_PONDERADO_ARESTA, this->IN_PONDERADO_VERTICE}};
    std::set<char> to_visit{ids_nos.begin(), ids_nos.end()};

    // start with the cheapest edge
    graph->createNode(std::get<0>(edges[0]), std::get<1>(edges[0]));
    graph->createNode(std::get<2>(edges[0]), std::get<3>(edges[0]));
    graph->createEdge(std::get<0>(edges[0]), std::get<2>(edges[0]), std::get<4>(edges[0]));
    to_visit.erase(std::get<0>(edges[0]));
    to_visit.erase(std::get<2>(edges[0]));

    std::map<char, std::tuple<char, int, char, int, int>> cheapest_edge;

    // fill cheapest_edge with all of the neighbors from one of the nodes
    for (const auto &edge : this->lista_adj.at(std::get<0>(edges[0]))->getArestas()) {
        cheapest_edge[edge->getIdNoAlvo()] = {
            edge->getIdNoAlvo(), this->lista_adj.at(edge->getIdNoAlvo())->getPeso(),
            std::get<0>(edges[0]), std::get<1>(edges[0]),
            edge->getPeso()
        };
    }

    // fill cheapest_edge with the neighbors from the other node that weren't already part of the map or
    // that are cheaper than with the previous node
    for (const auto &edge : this->lista_adj.at(std::get<2>(edges[0]))->getArestas()) {
        if (
            cheapest_edge.find(edge->getIdNoAlvo()) == cheapest_edge.end() ||
            edge->getPeso() < std::get<4>(cheapest_edge.at(edge->getIdNoAlvo()))
        ) {
            cheapest_edge[edge->getIdNoAlvo()] = {
                edge->getIdNoAlvo(), this->lista_adj.at(edge->getIdNoAlvo())->getPeso(),
                std::get<2>(edges[0]), std::get<3>(edges[0]),
                edge->getPeso()
            };
        }
    }

    while (!to_visit.empty()) {
        char currently_cheapest_node_id = *to_visit.begin();  // select any random node just to start with, if its not actually the cheapest it will get substituted

        // checks all nodes that are yet to be visited and selects the cheapest one
        for (auto node_id : to_visit) {
            if (
                cheapest_edge.find(node_id) != cheapest_edge.end() &&
                (cheapest_edge.find(currently_cheapest_node_id) == cheapest_edge.end() ||  // this is a failsafe in case the randomly selected node cannot currently be connected to the graph
                 std::get<4>(cheapest_edge.at(node_id)) < std::get<4>(cheapest_edge.at(currently_cheapest_node_id)))
            ) {
                currently_cheapest_node_id = node_id;
            }
        }

        // creates the new node and edge and removes it from the to_visit set
        graph->createNode(currently_cheapest_node_id, std::get<1>(cheapest_edge.at(currently_cheapest_node_id)));
        graph->createEdge(currently_cheapest_node_id, std::get<2>(cheapest_edge.at(currently_cheapest_node_id)), std::get<4>(cheapest_edge.at(currently_cheapest_node_id)));
        to_visit.erase(currently_cheapest_node_id);

        // checks if any of the new edges are cheaper than the currently stored ones
        for (const auto &edge : this->lista_adj.at(currently_cheapest_node_id)->getArestas()) {
            if (
                cheapest_edge.find(edge->getIdNoAlvo()) == cheapest_edge.end() ||
                edge->getPeso() < std::get<4>(cheapest_edge.at(edge->getIdNoAlvo()))
            ) {
                cheapest_edge[edge->getIdNoAlvo()] = {
                    edge->getIdNoAlvo(), this->lista_adj.at(edge->getIdNoAlvo())->getPeso(),
                    currently_cheapest_node_id, std::get<1>(cheapest_edge.at(currently_cheapest_node_id)),
                    edge->getPeso()
                };
            }
        }
    }

    return std::move(graph);
}

std::unique_ptr<Grafo> Grafo::arvoreGeradoraMinimaKruskal(std::vector<char> ids_nos) const {
    if (!this->IN_PONDERADO_ARESTA || this->IN_DIRECIONADO) {
        throw std::runtime_error("Tried generating a minimum spanning tree on an edge-unweighted or directed graph.\n");
    }

    if (ids_nos.empty()) {
        return std::unique_ptr<Grafo>{new Grafo{this->IN_DIRECIONADO, this->IN_PONDERADO_ARESTA, this->IN_PONDERADO_VERTICE}};
    }

    // this checks if all of the nodes selected for the minimum spanning tree are connected, if they aren't its not possible to make a
    // minimum spanning tree out of these nodes, since a minimum spanning tree can only be made on an undirected graph we only need
    // to check if node is connected to the rest, if it is that means all of the other ones also are, if it isn't that means that these
    // nodes can't form a connected graph
    auto direct_transitive_closure = this->fechoTransitivoDireto(ids_nos[0]);
    for (auto node_id : ids_nos) {
        if (std::find(direct_transitive_closure.begin(), direct_transitive_closure.end(), node_id) == direct_transitive_closure.end()) {
            throw std::runtime_error("Tried generating a minimum spanning tree while not all nodes on the graph are connected.\n");
        }
    }

    auto edges = this->getEdges(ids_nos);

    std::sort(  // sorts the list of edges
        edges.begin(),
        edges.end(),
        [](const std::tuple<char, int, char, int, int> &first, const std::tuple<char, int, char, int, int> &second) {  // sorts the vector by each edge's weight
            return std::get<4>(first) < std::get<4>(second);
        }
    );

    auto graph = std::unique_ptr<Grafo>{new Grafo{this->IN_DIRECIONADO, this->IN_PONDERADO_ARESTA, this->IN_PONDERADO_VERTICE}};
    for (auto edge : edges) {
        const auto &adjacency_list = graph->getListaDeAdjacencia();

        if (adjacency_list.find(std::get<0>(edge)) == adjacency_list.end()) {
            graph->createNode(std::get<0>(edge), std::get<1>(edge));
        }

        if (adjacency_list.find(std::get<2>(edge)) == adjacency_list.end()) {
            graph->createNode(std::get<2>(edge), std::get<3>(edge));
        }

        auto direct_transitive_closure = graph->fechoTransitivoDireto(std::get<0>(edge));
        if (std::find(direct_transitive_closure.begin(), direct_transitive_closure.end(), std::get<2>(edge)) == direct_transitive_closure.end()) {
            graph->createEdge(std::get<0>(edge), std::get<2>(edge), std::get<4>(edge));
        }
    }

    return std::move(graph);
}

// returns all edges for the given ids in a tuple vector, where each part of the tuples represents, respectively:
// first node's id, first node's weight, second node's id, second node's weight, edge's weight
std::vector<std::tuple<char, int, char, int, int>> Grafo::getEdges(const std::vector<char> &node_ids) const {
    std::vector<std::tuple<char, int, char, int, int>> edges;

    std::map<char, std::vector<char>> included_edges;
    for (const auto &node : this->lista_adj) {
        if (std::find(node_ids.begin(), node_ids.end(), node.first) == node_ids.end()) {
            continue;
        }

        auto &current_node_included_list = included_edges[node.first];

        for (const auto &edge : node.second->getArestas()) {
            if (std::find(node_ids.begin(), node_ids.end(), edge->getIdNoAlvo()) == node_ids.end()) {
                continue;
            }

            auto destination_position_in_included_edges = std::find(current_node_included_list.begin(), current_node_included_list.end(), edge->getIdNoAlvo());

            // this initially checks if the graph is directed, if it is there are no duplicate edges to be removed, so the second condition isn't checked
            // if it's not directed it then checks if the destination node doesn't exist in the already included edges vector to prevent duplicate edges from being included (i.e. (a, b) and (b, a))
            if (this->IN_DIRECIONADO || destination_position_in_included_edges == current_node_included_list.end()) {
                edges.push_back(
                    {node.first, node.second->getPeso(),
                     edge->getIdNoAlvo(), this->lista_adj.at(edge->getIdNoAlvo())->getPeso(),
                     edge->getPeso()}
                );
                included_edges[edge->getIdNoAlvo()].push_back(node.first);
            } else {
                // this instance is erased because there can be multiple edges that have the same starting and destination nodes,
                // if it wasn't erased they would be collpased into a single edge when printing
                current_node_included_list.erase(destination_position_in_included_edges);
            }
        }
    }

    return edges;
}

std::unique_ptr<Grafo> Grafo::arvoreCaminhamentoProfundidade(char id_no) const {
    auto graph = std::unique_ptr<Grafo>{new Grafo{false, false, false}};  // the resulting spanning tree is not directed nor weighted
    std::map<char, bool> has_been_visited;

    for (const auto &node : this->lista_adj) {
        has_been_visited[node.first] = false;
    }

    graph->createNode(id_no);
    this->depthFirstSearchHelper(id_no, has_been_visited, *graph);

    return std::move(graph);
}

void Grafo::depthFirstSearchHelper(char node_id, std::map<char, bool> &has_been_visited, Grafo &graph) const {
    has_been_visited[node_id] = true;

    for (const auto &edge : this->lista_adj.at(node_id)->getArestas()) {
        // creates node if it doesn't exist
        if (graph.lista_adj.find(edge->getIdNoAlvo()) == graph.lista_adj.end()) {
            graph.createNode(edge->getIdNoAlvo());
        }

        // creates an edge if it doesn't already exist
        if (
            std::find_if(
                graph.lista_adj[node_id]->getArestas().begin(),
                graph.lista_adj[node_id]->getArestas().end(),
                [&edge](const std::unique_ptr<Aresta> &comparison_edge) {
                    return comparison_edge->getIdNoAlvo() == edge->getIdNoAlvo();
                }
            ) == graph.lista_adj[node_id]->getArestas().end()
        ) {
            graph.createEdge(node_id, edge->getIdNoAlvo());
        }

        // checks if the node has already been visited
        if (!has_been_visited[edge->getIdNoAlvo()]) {
            this->depthFirstSearchHelper(edge->getIdNoAlvo(), has_been_visited, graph);
        }
    }
}

std::map<char, int> Grafo::getEccentricities() const {
    auto floyd = this->floydAllDistances();
    std::map<char, int> eccentricities;

    for (const auto &starting_node : floyd) {
        eccentricities[starting_node.first] = std::numeric_limits<int>::min();

        for (const auto &ending_node : floyd[starting_node.first]) {
            if (ending_node.second.first > eccentricities[starting_node.first]) {
                eccentricities[starting_node.first] = ending_node.second.first;
            }
        }
    }

    return eccentricities;
}

int Grafo::raio() const {
    auto eccentricities = this->getEccentricities();

    return this->raio(eccentricities);
}

int Grafo::raio(const std::map<char, int> &eccentricities) const {
    int minimum_eccentricity = std::numeric_limits<int>::max();

    for (const auto &eccentricity : eccentricities) {
        if (eccentricity.second != std::numeric_limits<int>::min() && eccentricity.second < minimum_eccentricity) {
            minimum_eccentricity = eccentricity.second;
        }
    }

    if (minimum_eccentricity != std::numeric_limits<int>::max()) {
        return minimum_eccentricity;
    } else {
        throw std::runtime_error("Tried finding the radius of a disconnected graph.\n");
    }
}

int Grafo::diametro() const {
    auto eccentricities = this->getEccentricities();

    return this->diametro(eccentricities);
}

int Grafo::diametro(const std::map<char, int> &eccentricities) const {
    int maximum_eccentricity = std::numeric_limits<int>::min();

    for (const auto &eccentricity : eccentricities) {
        if (eccentricity.second > maximum_eccentricity) {
            maximum_eccentricity = eccentricity.second;
        }
    }

    if (maximum_eccentricity != std::numeric_limits<int>::min()) {
        return maximum_eccentricity;
    } else {
        throw std::runtime_error("Tried finding the diameter of a disconnected graph.\n");
    }
}

std::vector<char> Grafo::centro() const {
    auto eccentricities = this->getEccentricities();

    return this->centro(eccentricities);
}

std::vector<char> Grafo::centro(const std::map<char, int> &eccentricities) const {
    std::vector<char> central_nodes;
    auto radius = this->raio(eccentricities);

    for (const auto &node : eccentricities) {
        if (node.second == radius) {
            central_nodes.push_back(node.first);
        }
    }

    return central_nodes;
}

std::vector<char> Grafo::periferia() const {
    auto eccentricities = this->getEccentricities();

    return this->periferia(eccentricities);
}

std::vector<char> Grafo::periferia(const std::map<char, int> &eccentricities) const {
    std::vector<char> peripheral_nodes;
    auto diameter = this->diametro(eccentricities);

    for (const auto &node : eccentricities) {
        if (node.second == diameter) {
            peripheral_nodes.push_back(node.first);
        }
    }

    return peripheral_nodes;
}

std::vector<std::tuple<char, int, char, int, int>> Grafo::getEdges() const {
    std::vector<char> keys;
    for (const auto &node_id : this->lista_adj) {
        keys.push_back(node_id.first);
    }

    return this->getEdges(keys);
}

// modified version of the getEdges helper function, returning a vector of a pair of tuples
// ordered based on the sum of the degrees of the nodes on each end of the respective edge
// each part of the pair of tuples represents, respectively:
// {
//  {first node's id, first node's weight, second node's id, second node's weight, edge's weight}, // tuple values
//  sum of the degrees of the nodes on each end of the respective edge
// }
std::vector<std::pair<std::tuple<char, int, char, int, int>, int>> Grafo::getSortedEdgeVector() const {
    std::vector<std::pair<std::tuple<char, int, char, int, int>, int>> edges;

    std::map<char, std::vector<char>> included_edges;
    for (const auto &node : this->lista_adj) {
        auto &current_node_included_list = included_edges[node.first];

        for (const auto &edge : node.second->getArestas()) {
            auto destination_position_in_included_edges = std::find(current_node_included_list.begin(), current_node_included_list.end(), edge->getIdNoAlvo());

            // this initially checks if the graph is directed, if it is there are no duplicate edges to be removed, so the second condition isn't checked
            // if it's not directed it then checks if the destination node doesn't exist in the already included edges vector to prevent duplicate edges from being included (i.e. (a, b) and (b, a))
            if (this->IN_DIRECIONADO || destination_position_in_included_edges == current_node_included_list.end()) {
                edges.push_back(
                    {{node.first, node.second->getPeso(),
                      edge->getIdNoAlvo(), this->lista_adj.at(edge->getIdNoAlvo())->getPeso(),
                      edge->getPeso()},
                     int(node.second->getArestas().size() + this->lista_adj.at(edge->getIdNoAlvo())->getArestas().size())
                    }
                );
                included_edges[edge->getIdNoAlvo()].push_back(node.first);
            } else {
                // this instance is erased because there can be multiple edges that have the same starting and destination nodes,
                // if it wasn't erased they would be collpased into a single edge when printing
                current_node_included_list.erase(destination_position_in_included_edges);
            }
        }
    }

    // sorts the list of edges in descending order
    std::sort(edges.begin(), edges.end(), [](const auto &first, const auto &second) {
        return first.second > second.second;
    });

    return edges;
}
