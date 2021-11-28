#include <iostream>
#include <vector>

using VertexT = int32_t;
using RankT = int32_t;
using WeightT = int32_t;

class DSU {
public:
    explicit DSU(int32_t vertexes_num) {
        parents_.resize(vertexes_num + 1);
        ranks_.resize(vertexes_num + 1);
        weights_.resize(vertexes_num + 1);
        for (int32_t i = 0; i < vertexes_num; ++i) {
            MakeSet(i);
        }
    }

    void MakeSet(const VertexT& vertex) {
        parents_[vertex] = vertex;
        ranks_[vertex] = 1;
        weights_[vertex] = 1;
    }

    VertexT FindSet(const VertexT& vertex) {
        if (vertex == parents_[vertex]) {
            return vertex;
        }
        parents_[vertex] = FindSet(parents_[vertex]);
        return parents_[vertex];
    }

    WeightT FindWeight(const VertexT& vertex) {
        if (vertex == parents_[vertex]) {
            return weights_[vertex];
        }
        return FindWeight(parents_[vertex]);
    }

    void UnionSets(const VertexT& first, const VertexT& second) {
        VertexT first_parent = FindSet(first);
        VertexT second_parent = FindSet(second);
        if (first_parent != second_parent) {
            if (ranks_[first_parent] < ranks_[second_parent]) {
                std::swap(first_parent, second_parent);
            }
            parents_[second_parent] = first_parent;
            if (ranks_[second_parent] == ranks_[first_parent]) {
                ++ranks_[first_parent];
            }
            weights_[first_parent] += weights_[second_parent];
        }
    }

private:
    std::vector<VertexT> parents_;
    std::vector<RankT> ranks_;
    std::vector<WeightT> weights_;
};

int main() {
    int32_t lands_num = 0;
    int32_t bridges_num = 0;
    std::cin >> lands_num >> bridges_num;

    DSU country(lands_num);
    int32_t min_bridges = 0;

    for (int32_t i = 0; i < bridges_num; ++i) {
        VertexT first = 0;
        VertexT second = 0;
        std::cin >> first >> second;
        country.UnionSets(first, second);
        if (country.FindWeight(first) == lands_num) {
            min_bridges = i + 1;
            break;
        }
    }

    std::cout << min_bridges << std::endl;

    return 0;
}