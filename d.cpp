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

const WeightT kMAXWEIGHT = 1'000'000;

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
    //    virtual const std::vector<Edge>& GetEdges(const VertexT& vert) const;
    virtual void InsertEdge(const VertexT& x, const VertexT& y, const WeightT& weight = 0) = 0;

protected:
    size_t vertexes_num_;
    size_t edges_num_;
    bool is_oriented_ = false;
};

class GraphTable final : public IGraph {
public:
    explicit GraphTable(size_t vertexes_num, size_t edges_num = 0, bool is_oriented = false) {
        vertexes_num_ = vertexes_num;
        edges_num_ = edges_num;
        is_oriented_ = is_oriented;
        std::vector<WeightT> nulls_line(vertexes_num_ + 1, 0);
        adjacency_table_.resize(vertexes_num_ + 1, nulls_line);
    }

    void InsertEdge(const VertexT& first, const VertexT& second, const WeightT& weight = 0) override {
        if (is_oriented_) {
            InsertEdgeOriented(first, second, weight);
        } else {
            InsertEdgeNotOriented(first, second, weight);
        }
        if (weight > 0) {
            edges_num_++;
        }
    }

    std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const override {
        std::vector<VertexT> answer = {};
        for (VertexT current_vertex = 0; current_vertex < static_cast<VertexT>(vertexes_num_); ++current_vertex) {
            if (adjacency_table_[vert][current_vertex] > 0) {
                answer.push_back(current_vertex);
            }
        }
        return answer;
    }

    std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const override {
        std::vector<WeightT> answer = {};
        for (VertexT current_vertex = 0; current_vertex < static_cast<VertexT>(vertexes_num_); ++current_vertex) {
            answer.push_back(adjacency_table_[vert][current_vertex]);
        }
        return answer;
    }

    std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const override {
        std::vector<GraphNeighboursNode> answer = {};
        for (VertexT current_vertex = 0; current_vertex < static_cast<VertexT>(vertexes_num_); ++current_vertex) {
            if (current_vertex == vert) {
                continue;
            }
            answer.push_back({current_vertex, adjacency_table_[vert][current_vertex]});
        }
        return answer;
    }

private:
    std::vector<std::vector<WeightT>> adjacency_table_;

    void InsertEdgeOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 1) {
        adjacency_table_[first][second] = weight;
    }

    void InsertEdgeNotOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 1) {
        adjacency_table_[first][second] = weight;
        adjacency_table_[second][first] = weight;
    }
};

IGraph::VertexColoursT SwapColour(IGraph::VertexColoursT current_colour) {
    return (current_colour == IGraph::GREY_COLOUR) ? IGraph::BLACK_COLOUR : IGraph::GREY_COLOUR;
}

class DSU {
public:
    explicit DSU(int32_t vertexes_num) {
        parents_.resize(vertexes_num + 1);
        ranks_.resize(vertexes_num + 1);
        weights_.resize(vertexes_num + 1);
        for (int32_t i = 0; i < vertexes_num; ++i) {
            MakeSet(i);
        }
        number_ = vertexes_num;
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
            --number_;
        }
        weights_[first_parent] += weight;
    }

    int32_t GetSetNumber() {
        return number_;
    }

private:
    int32_t number_;
    std::vector<VertexT> parents_;
    std::vector<RankT> ranks_;
    std::vector<WeightT> weights_;
};

const int32_t IGraph::kUNDEFINED = INT_MAX;

WeightT PrimMstFindArray(const IGraph& graph) {
    std::vector<WeightT> dist(graph.GetVertexesNum() + 1, IGraph::kUNDEFINED);
    std::vector<VertexT> prev(graph.GetVertexesNum() + 1, -1);
    std::set<VertexT> mst = {};
    WeightT answer = 0;
    dist[1] = 0;

    while (mst.size() != graph.GetVertexesNum()) {
        VertexT current_vertex = std::min_element(dist.begin(), dist.end()) - dist.begin();
        mst.insert(current_vertex);

        if (prev[current_vertex] != -1) {
            answer += dist[current_vertex];
        }
        dist[current_vertex] = IGraph::kUNDEFINED;

        for (auto current_neighbour : graph.GetNeighbours(current_vertex)) {
            if (mst.find(current_neighbour.vertex_) == mst.end() &&
                dist[current_neighbour.vertex_] > current_neighbour.weight_) {
                prev[current_neighbour.vertex_] = current_vertex;
                dist[current_neighbour.vertex_] = current_neighbour.weight_;
            }
        }
    }

    return answer;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t vertexes_num = 0;
    std::cin >> vertexes_num;
    GraphTable graph(vertexes_num + 1);  // 1 for extra_vertex

    for (int32_t i = 1; i < vertexes_num + 1; ++i) {
        for (int32_t j = 1; j < vertexes_num + 1; ++j) {
            WeightT value = 0;
            std::cin >> value;
            graph.InsertEdge(i, j, value);
        }
    }

    for (int32_t i = 1; i < vertexes_num + 1; ++i) {
        WeightT weight = 0;
        std::cin >> weight;
        graph.InsertEdge(0, i, weight);
    }

    std::cout << PrimMstFindArray(graph) << std::endl;

    return 0;
}