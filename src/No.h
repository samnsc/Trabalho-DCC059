#ifndef __NO_H__
#define __NO_H__

#include <memory>
#include <vector>

#include "Aresta.h"

class No {
    public:
        No(char id, int peso);

        void createEdge(char destination_id, int weight);

        char getId() const;
        int getPeso() const;
        const std::vector<std::unique_ptr<Aresta>>& getArestas() const;

    private:
        const char ID;
        const int PESO;
        std::vector<std::unique_ptr<Aresta>> arestas;
};

#endif
