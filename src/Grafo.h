#ifndef __GRAFO_H__
#define __GRAFO_H__

#include <vector>

#include "No.h"

class Grafo {
    public:
        inline Grafo() {};

        inline ~Grafo() {
            for (auto vector : lista_adj) {
                delete vector;
            }
        };

        std::vector<char> fechoTransitivoDireto(char id_no);                  // a
        std::vector<char> fechoTransitivoIndireto(char id_no);                // b
        std::vector<char> caminhoMinimoDijkstra(char id_no_a, char id_no_b);  // c
        std::vector<char> caminhoMinimoFloyd(char id_no, char id_no_b);       // d
        Grafo* arvoreGeradoraMinimaPrim(std::vector<char> ids_nos);           // e
        Grafo* arvoreGeradoraMinimaKruskal(std::vector<char> ids_nos);        // f
        Grafo* arvoreCaminhamentoProfundidade(char id_no);                    // g
        int raio();                                                           // h 1
        int diametro();                                                       // h 2
        std::vector<char> centro();                                           // h 3
        std::vector<char> periferia();                                        // h 4

        int ordem;
        bool in_direcionado;
        bool in_ponderado_aresta;
        bool in_ponderado_vertice;
        std::vector<No*> lista_adj;
};

#endif
