#include "Grafo.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
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

std::vector<char> Grafo::caminhoMinimoDijkstra(char id_no_a, char id_no_b) {
    std::cout << "Metodo nao implementado" << std::endl;
    return {};
}

std::vector<char> Grafo::caminhoMinimoFloyd(char id_no, char id_no_b) {
    std::cout << "Metodo nao implementado" << std::endl;
    return {};
}

Grafo *Grafo::arvoreGeradoraMinimaPrim(std::vector<char> ids_nos) {
    std::cout << "Metodo nao implementado" << std::endl;
    return nullptr;
}

Grafo *Grafo::arvoreGeradoraMinimaKruskal(std::vector<char> ids_nos) {
    std::cout << "Metodo nao implementado" << std::endl;
    return nullptr;
}

Grafo *Grafo::arvoreCaminhamentoProfundidade(char id_no) {
    std::cout << "Metodo nao implementado" << std::endl;
    return nullptr;
}

int Grafo::raio() {
    std::cout << "Metodo nao implementado" << std::endl;
    return 0;
}

int Grafo::diametro() {
    std::cout << "Metodo nao implementado" << std::endl;
    return 0;
}

std::vector<char> Grafo::centro() {
    std::cout << "Metodo nao implementado" << std::endl;
    return {};
}

std::vector<char> Grafo::periferia() {
    std::cout << "Metodo nao implementado" << std::endl;
    return {};
}
