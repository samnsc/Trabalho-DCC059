#include "No.h"

#include <memory>

// a delegating constructor is used here because const class members have to be
// instantiated before the initializer
No::No(char id, int peso): ID{id}, PESO{peso} {}

void No::createEdge(char destination_id, int weight) {
    // no checks are done here for if this edge already exists since multiple edges
    // can point to the same destination node
    this->arestas.push_back(std::unique_ptr<Aresta>{new Aresta{destination_id, weight}});
}

char No::getId() const {
    return this->ID;
}

int No::getPeso() const {
    return this->PESO;
}

const std::vector<std::unique_ptr<Aresta>>& No::getArestas() const {
    return this->arestas;
}
