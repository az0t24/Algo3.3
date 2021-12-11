#include <iostream>
#include <vector>

using WeightT = int32_t;
using VertexT = int32_t;

template <typename T>
class FunctorSum {
public:
    T operator()(const T& first, const T& second) const {
        return first + second;
    }
};

template <typename ElemT, typename CompValueT, typename Functor>
class DSU {
public:
    explicit DSU(int32_t vertexes_num) {
        parents_.resize(vertexes_num + 1);
        ranks_.resize(vertexes_num + 1);
        comp_values_.resize(vertexes_num + 1);
        for (int32_t i = 0; i < vertexes_num; ++i) {
            MakeSet(i);
        }
    }

    void MakeSet(const ElemT& vertex) {
        parents_[vertex] = vertex;
        ranks_[vertex] = 1;
        comp_values_[vertex] = 0;
    }

    CompValueT FindWeight(const ElemT& vertex) {
        if (vertex == parents_[vertex]) {
            return comp_values_[vertex];
        }
        return FindWeight(parents_[vertex]);
    }

    bool IsInOneComp(const ElemT& first, const ElemT& second) {
        return FindSet(first) == FindSet(second);
    }

    void UnionSets(const ElemT& first, const ElemT& second, const CompValueT& comp_value = 0) {
        ElemT first_parent = FindSet(first);
        ElemT second_parent = FindSet(second);
        if (first_parent != second_parent) {
            if (ranks_[first_parent] < ranks_[second_parent]) {
                std::swap(first_parent, second_parent);
            }
            parents_[second_parent] = first_parent;
            if (ranks_[second_parent] == ranks_[first_parent]) {
                ++ranks_[first_parent];
            }
            comp_values_[first_parent] = func_(comp_values_[second_parent], comp_values_[first_parent]);
        }
        comp_values_[first_parent] = func_(comp_value, comp_values_[first_parent]);
    }

private:
    using RankT = int32_t;
    std::vector<ElemT> parents_;
    std::vector<RankT> ranks_;
    std::vector<CompValueT> comp_values_;
    Functor func_;

    ElemT FindSet(const ElemT& vertex) {
        if (vertex == parents_[vertex]) {
            return vertex;
        }
        parents_[vertex] = FindSet(parents_[vertex]);
        return parents_[vertex];
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t element_num = 0;
    int32_t request_num = 0;
    std::cin >> element_num >> request_num;

    DSU<VertexT, WeightT, FunctorSum<WeightT>> sets(element_num);

    for (int32_t i = 0; i < request_num; ++i) {
        int32_t command = 0;
        std::cin >> command;
        if (command == 1) {
            VertexT first = 0;
            VertexT second = 0;
            WeightT weight = 0;
            std::cin >> first >> second >> weight;
            sets.UnionSets(first, second, weight);
        } else {
            VertexT vertex = 0;
            std::cin >> vertex;
            std::cout << sets.FindWeight(vertex) << '\n';
        }
    }

    return 0;
}