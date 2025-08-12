#include "Gulosos.h"

#include <algorithm>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#include "Grafo.h"

// a delegating constructor is used here because all class members have to be initialized
// before the class initializer's body, since Grafo does not have a default constructor
//
Gulosos::Gulosos(Grafo *graph) {
    this->graph = graph;
}

bool Gulosos::isAdjacentToDominatingEdge(const std::tuple<char, int, char, int, int> &edge, const std::vector<std::tuple<char, int, char, int, int>> &dominating_edges) {
    // if any of the vertices are shared between a dominating and a non-dominating edge
    // that means that the non-dominating edge is being dominatend
    for (const auto &dominating_edge : dominating_edges) {
        if (
            std::get<0>(dominating_edge) == std::get<0>(edge) ||
            std::get<0>(dominating_edge) == std::get<2>(edge) ||
            std::get<2>(dominating_edge) == std::get<0>(edge) ||
            std::get<2>(dominating_edge) == std::get<2>(edge)
        ) {
            return true;
        }
    }

    return false;
}

std::vector<std::tuple<char, int, char, int, int>> Gulosos::greedyEdgeDominatingSet() const {
    // solution
    std::vector<std::tuple<char, int, char, int, int>> solution;

    // candidates
    auto sorted_edges = this->graph->getSortedEdgeVector();

    for (const auto &current_edge : sorted_edges) {
        if (!Gulosos::isAdjacentToDominatingEdge(current_edge.first, solution)) {
            solution.push_back(current_edge.first);
        }
    }

    return solution;
}

void Gulosos::removeDominatedCandidates(const std::tuple<char, int, char, int, int> &edge, std::vector<std::pair<std::tuple<char, int, char, int, int>, int>> &candidates) {
    auto new_end = std::remove_if(
        candidates.begin(), candidates.end(),
        [edge](const auto &candidate) {  // edge has to be captured by value because it will otherwise the value it currently points to will be moved by the remove_if function, causing it to point to the wrong value
            return (
                std::get<0>(candidate.first) == std::get<0>(edge) ||
                std::get<0>(candidate.first) == std::get<2>(edge) ||
                std::get<2>(candidate.first) == std::get<0>(edge) ||
                std::get<2>(candidate.first) == std::get<2>(edge)
            );
        }
    );

    candidates.erase(new_end, candidates.end());
}

std::vector<std::tuple<char, int, char, int, int>> Gulosos::greedyRandomizedAdaptiveEdgeDominatingSet(float alpha, unsigned int iteration_amount, unsigned int seed) const {
    std::vector<std::tuple<char, int, char, int, int>> best_solution;

    for (int i = 0; i < iteration_amount; i++) {
        // solution
        std::vector<std::tuple<char, int, char, int, int>> solution;

        // candidates
        auto sorted_edges = this->graph->getSortedEdgeVector();

        std::mt19937 generator{seed};
        int upper_limit = static_cast<int>(alpha * sorted_edges.size() - 1);
        if (upper_limit < 0) {
            upper_limit = 0;
        }
        std::uniform_int_distribution<> uniform_distribution{0, upper_limit};

        // select a random edge from a specific interval
        auto current_edge = sorted_edges.begin() + uniform_distribution(generator);
        while (current_edge != sorted_edges.end()) {  // when the solutions vector is empty the beginning and ending iterators will be equal
            // there's no need to check if the current edge is already being dominated since
            // dominated edges are not going to be part of the solutions vector
            solution.push_back((*current_edge).first);

            Gulosos::removeDominatedCandidates((*current_edge).first, sorted_edges);

            int upper_limit = static_cast<int>(alpha * sorted_edges.size() - 1);
            if (upper_limit < 0) {
                upper_limit = 0;
            }
            uniform_distribution = std::uniform_int_distribution<>{0, upper_limit};

            int val = uniform_distribution(generator);
            current_edge = sorted_edges.begin() + val;
        }

        if (solution.size() < best_solution.size() || best_solution.empty()) {
            best_solution = solution;
        }
    }

    return best_solution;
}

void Gulosos::updateProbabilities(const std::map<float, std::tuple<int, float>> &solutions, std::map<float, float> &alpha_cumulative_probabilities, const std::vector<float> &alpha_values, float best_result) {
    std::map<float, float> alpha_qualities;

    float alpha_qualities_sum = 0;
    for (auto alpha : alpha_values) {
        float current_alpha_quality = best_result / std::get<1>(solutions.at(alpha));

        alpha_qualities[alpha] = current_alpha_quality;
        alpha_qualities_sum += current_alpha_quality;
    }

    float probability_sum = 0;
    for (auto alpha : alpha_values) {
        float current_alpha_probability = alpha_qualities[alpha] / alpha_qualities_sum;

        alpha_cumulative_probabilities[alpha] = current_alpha_probability + probability_sum;
        probability_sum += current_alpha_probability;
    }
}

std::vector<std::tuple<char, int, char, int, int>> Gulosos::greedyRandomizedAdaptiveReactiveEdgeDominatingSet(const std::vector<float> &alpha_values, unsigned int reactive_iteration_amount, unsigned int randomized_iteration_amount, unsigned int seed) const {
    std::vector<std::tuple<char, int, char, int, int>> best_solution;

    // the tuple represents, respectively:
    // amount of iterations, result average
    std::map<float, std::tuple<int, float>> solutions;
    float best_result = std::numeric_limits<float>::max();

    // run the algorithm once for each alpha value to get data for initial probabilities
    for (auto alpha : alpha_values) {
        auto solution = this->greedyRandomizedAdaptiveEdgeDominatingSet(alpha, randomized_iteration_amount, seed);

        if (solution.size() < best_result) {
            best_result = solution.size();
            best_solution = solution;
        }

        std::get<0>(solutions[alpha]) = 1;
        std::get<1>(solutions[alpha]) = solution.size();
    }

    std::mt19937 generator{seed};
    std::uniform_real_distribution<float> uniform_distribution{0.0, 1.0};

    std::map<float, float> alpha_cumulative_probabilities;
    for (int i = 0; i < reactive_iteration_amount; i++) {
        Gulosos::updateProbabilities(solutions, alpha_cumulative_probabilities, alpha_values, best_result);

        float random_value = uniform_distribution(generator);
        auto selected_alpha_iterator = alpha_cumulative_probabilities.lower_bound(random_value);
        if (selected_alpha_iterator == alpha_cumulative_probabilities.end()) {
            selected_alpha_iterator--;
        }
        float selected_alpha = (*selected_alpha_iterator).second;

        auto solution = this->greedyRandomizedAdaptiveEdgeDominatingSet(selected_alpha, randomized_iteration_amount, seed);

        // if the current solution is better than the current best solution than replace that value
        if (solution.size() < best_result) {
            best_result = solution.size();
            best_solution = solution;
        }

        // multiply the amount of iterations by the result average
        auto sum_of_results = std::get<0>(solutions[selected_alpha]) * std::get<1>(solutions[selected_alpha]);

        std::get<0>(solutions[selected_alpha]) = std::get<0>(solutions[selected_alpha]) + 1;
        std::get<1>(solutions[selected_alpha]) = (sum_of_results + solution.size()) / std::get<0>(solutions[selected_alpha]);
    }

    return best_solution;
}

bool Gulosos::checkResultValidity(std::vector<std::tuple<char, int, char, int, int>> results) const {
    std::set<char> nodes_adjacent_to_dominated_edges;

    for (const auto &result : results) {
        nodes_adjacent_to_dominated_edges.insert(std::get<0>(result));
        nodes_adjacent_to_dominated_edges.insert(std::get<2>(result));
    }

    auto all_edges = this->graph->getEdges();

    for (const auto &edge : all_edges) {
        if (
            nodes_adjacent_to_dominated_edges.find(std::get<0>(edge)) == nodes_adjacent_to_dominated_edges.end() &&
            nodes_adjacent_to_dominated_edges.find(std::get<2>(edge)) == nodes_adjacent_to_dominated_edges.end()
        ) {
            return false;
        }
    }

    return true;
}
