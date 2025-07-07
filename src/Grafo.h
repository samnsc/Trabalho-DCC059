#ifndef __GRAFO_H__
#define __GRAFO_H__

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "No.h"

class Grafo {
    public:
        Grafo(bool is_directed, bool has_weighted_edges, bool has_weighted_nodes);

        void createNode(char id, int weight = 0);
        void createEdge(char starting_id, char destination_id, int weight = 0);

        const std::map<char, std::unique_ptr<No>>& getListaDeAdjacencia() const;
        int getOrdem() const;

        void printGraph() const;
        void printAdjacencyList() const;
        std::string formattedAdjacencyList() const;

        std::vector<char> fechoTransitivoDireto(char id_no) const;                      // a
        std::vector<char> fechoTransitivoIndireto(char id_no) const;                    // b
        std::vector<char> caminhoMinimoDijkstra(char id_no_a, char id_no_b) const;      // c
        std::vector<char> caminhoMinimoFloyd(char id_no_a, char id_no_b) const;         // d
        std::unique_ptr<Grafo> arvoreGeradoraMinimaPrim(std::vector<char> ids_nos);     // e
        std::unique_ptr<Grafo> arvoreGeradoraMinimaKruskal(std::vector<char> ids_nos);  // f
        std::unique_ptr<Grafo> arvoreCaminhamentoProfundidade(char id_no);              // g

        int raio();                     // h 1
        int diametro();                 // h 2
        std::vector<char> centro();     // h 3
        std::vector<char> periferia();  // h 4

    private:
        const bool IN_DIRECIONADO;
        const bool IN_PONDERADO_ARESTA;
        const bool IN_PONDERADO_VERTICE;

        std::map<char, std::unique_ptr<No>> lista_adj;
        int ordem;

        void directTransitiveClosureHelper(const std::unique_ptr<No>& node, std::vector<char>& direct_transitive_closure) const;
        bool indirectTransitiveClosureHelper(char target_node_id, const std::unique_ptr<No>& current_node, const std::vector<char>& indirect_transitive_closure, const std::vector<char>& cant_reach_target_node, std::vector<char>& scoured_nodes) const;
        void dijkstraShortestPathHelper(const std::tuple<char, int, bool, std::vector<char>>& current_node, int summed_distance, std::vector<std::tuple<char, int, bool, std::vector<char>>>& distance_to_node) const;
        std::map<char, std::map<char, std::pair<int, std::vector<char>>>> floydAllDistances() const;
        std::vector<std::tuple<char, int, char, int, int>> getEdges(const std::vector<char>& node_ids) const;
};

#endif
