#ifndef __GULOSOS_H__
#define __GULOSOS_H__

#include <tuple>
#include <vector>

#include "Grafo.h"

class Gulosos {
    public:
        Gulosos(Grafo* graph);

        std::vector<std::tuple<char, int, char, int, int>> greedyEdgeDominatingSet() const;
        std::vector<std::tuple<char, int, char, int, int>> greedyRandomizedAdaptiveEdgeDominatingSet(float alpha, unsigned int iteration_amount, unsigned int seed) const;
        std::vector<std::tuple<char, int, char, int, int>> greedyRandomizedAdaptiveReactiveEdgeDominatingSet(const std::vector<float>& alpha_values, unsigned int reactive_iteration_amount, unsigned int randomized_iteration_amount, unsigned int seed) const;

        bool checkResultValidity(std::vector<std::tuple<char, int, char, int, int>> results) const;

    private:
        Grafo* graph;

        static void updateProbabilities(const std::map<float, std::tuple<int, float>>& solutions, std::map<float, float>& alpha_cumulative_probabilities, const std::vector<float>& alpha_values, float best_result);
        static void removeDominatedCandidates(const std::tuple<char, int, char, int, int>& edge, std::vector<std::pair<std::tuple<char, int, char, int, int>, int>>& candidates);
        static bool isAdjacentToDominatingEdge(const std::tuple<char, int, char, int, int>& edge, const std::vector<std::tuple<char, int, char, int, int>>& dominating_edges);
};

#endif
