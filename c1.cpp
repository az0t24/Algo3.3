#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <queue>
#include <set>

using DistT = int32_t;
using VertexT = int32_t;
using WeightT = int32_t;
using RankT = int32_t;

struct Point {
    int32_t x_;
    int32_t y_;

    bool operator==(const Point& second) {
        return x_ == second.x_ && y_ == second.y_;
    }
};

class IGraph {
public:
    struct GraphNeighboursNode {
        VertexT vertex_;
        WeightT weight_;

        friend bool operator<(const GraphNeighboursNode& first, const GraphNeighboursNode& second) {
            return first.weight_ < second.weight_ ||
                   (first.weight_ == second.weight_ && first.vertex_ < second.vertex_);
        }
    };

    struct Edge {
        VertexT from_;
        VertexT to_;
        WeightT weight_;
        int32_t number_;

        explicit Edge(const VertexT& first, const VertexT& second, const VertexT& weight) {
            from_ = first;
            to_ = second;
            weight_ = weight;
        }

        friend bool operator<(const Edge& first, const Edge& second) {
            return first.from_ < second.from_ || (first.from_ == second.from_ && first.to_ <= second.to_);
        }

        friend bool operator==(const Edge& first, const Edge& second) {
            return first.from_ == second.from_ && first.to_ == second.to_ && first.weight_ == second.weight_;
        }
    };

    using VertexColoursT = enum {
        WHITE_COLOUR = 0,
        GREY_COLOUR = 1,
        BLACK_COLOUR = 2,
    };

    static const int32_t kUNDEFINED;

    size_t GetVertexesNum() const {
        return vertexes_num_;
    }

    size_t GetEdgesNum() const {
        return edges_num_;
    }

    virtual std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const = 0;
    virtual std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const = 0;
    virtual std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const = 0;
    virtual void InsertEdge(const VertexT& x, const VertexT& y, const WeightT& weight = 0) = 0;

protected:
    size_t vertexes_num_;
    size_t edges_num_;
    bool is_oriented_ = false;
};

class GraphList final : public IGraph {
public:
    explicit GraphList(size_t vertexes_num, size_t edges_num = 0, bool is_oriented = false) {
        vertexes_num_ = vertexes_num;
        edges_num_ = edges_num;
        is_oriented_ = is_oriented;
        adjacency_list_.resize(vertexes_num_ + 1, {});
    }

    void InsertEdge(const VertexT& first, const VertexT& second, const WeightT& weight = 0) override {
        if (is_oriented_) {
            InsertEdgeOriented(first, second, weight);
        } else {
            InsertEdgeNotOriented(first, second, weight);
        }
    }

    std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const override {
        std::vector<VertexT> answer(adjacency_list_[vert].size());
        for (size_t i = 0; i < answer.size(); ++i) {
            answer[i] = adjacency_list_[vert][i].to_;
        }
        return answer;
    }

    std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const override {
        std::vector<WeightT> answer(adjacency_list_[vert].size());
        for (size_t i = 0; i < answer.size(); ++i) {
            answer[i] = adjacency_list_[vert][i].weight_;
        }
        return answer;
    }

    std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const override {
        std::vector<GraphNeighboursNode> answer(adjacency_list_[vert].size());
        for (size_t i = 0; i < answer.size(); ++i) {
            answer[i].vertex_ = adjacency_list_[vert][i].to_;
            answer[i].weight_ = adjacency_list_[vert][i].weight_;
        }
        return answer;
    }

private:
    std::vector<std::vector<Edge>> adjacency_list_;  // using adjacency list

    void InsertEdgeOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 0) {
        adjacency_list_[first].emplace_back(first, second, weight);
    }

    void InsertEdgeNotOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 0) {
        adjacency_list_[first].emplace_back(first, second, weight);
        adjacency_list_[second].emplace_back(second, first, weight);
    }
};

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
        weights_[vertex] = 0;
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

    void UnionSets(const VertexT& first, const VertexT& second, const WeightT& weight = 0) {
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
        weights_[first_parent] += weight;
    }

private:
    std::vector<VertexT> parents_;
    std::vector<RankT> ranks_;
    std::vector<WeightT> weights_;
};

const int32_t IGraph::kUNDEFINED = INT_MAX;

WeightT KruskalMstFind(const std::vector<IGraph::Edge>& edges, const int32_t vertexes_num) {
    WeightT answer = 0;

    DSU comps(vertexes_num);

    for (size_t i = 0; i < edges.size(); ++i) {
        if (comps.FindSet(edges[i].to_) != comps.FindSet(edges[i].from_)) {
            comps.UnionSets(edges[i].to_, edges[i].from_);
            answer += edges[i].weight_;
        }
    }

    return answer;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t vertexes_num = 0;
    int32_t edges_num = 0;
    std::cin >> vertexes_num >> edges_num;
    std::vector<IGraph::Edge> edges;

    for (int32_t i = 0; i < edges_num; ++i) {
        VertexT first = 0;
        VertexT second = 0;
        WeightT value = 0;
        std::cin >> first >> second >> value;
        edges.emplace_back(first, second, value);
    }

    std::cout << KruskalMstFind(edges, vertexes_num) << std::endl;

    return 0;
}