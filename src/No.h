#ifndef __NO_H__
#define __NO_H__

#include <vector>

#include "Aresta.h"

class No {
    public:
        inline No() {};

        inline ~No() {
            for (auto aresta : arestas) {
                delete aresta;
            }
        };

        char id;
        int peso;
        std::vector<Aresta *> arestas;
};

#endif
