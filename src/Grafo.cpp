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

std::vector<char> Grafo::fechoTransitivoDireto(char id_no) {
    std::cout << "Metodo nao implementado" << std::endl;
    return {};
}

std::vector<char> Grafo::fechoTransitivoIndireto(char id_no) {
    std::cout << "Metodo nao implementado" << std::endl;
    return {};
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
