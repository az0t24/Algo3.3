#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <queue>
#include <set>

using DistT = int32_t;
using VertexT = int32_t;
using WeightT = int64_t;

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
        WeightT time_in_;
        WeightT time_out_;
        int32_t number_;
        WeightT flow_;
        WeightT flow_capacity_;

        Edge(const VertexT& first, const VertexT& second, const VertexT& weight) {
            from_ = first;
            to_ = second;
            weight_ = weight;
            flow_capacity_ = weight;
            flow_ = 0;
            number_ = 0;
        }

        Edge(const VertexT& first, const VertexT& second, const WeightT& time_in, const WeightT& time_out) {
            from_ = first;
            to_ = second;
            time_in_ = time_in;
            time_out_ = time_out;
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

    static const int64_t kUNDEFINED;

    size_t GetVertexesNum() const {
        return vertexes_num_;
    }

    size_t GetEdgesNum() const {
        return edges_num_;
    }

    virtual std::vector<std::vector<WeightT>>& GetEdges() = 0;
    virtual std::vector<std::vector<WeightT>> GetEdges() const = 0;
    virtual std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const = 0;
    virtual std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const = 0;
    virtual std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const = 0;
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

    std::vector<std::vector<WeightT>>& GetEdges() override {
        return adjacency_table_;
    }

    std::vector<std::vector<WeightT>> GetEdges() const override {
        return adjacency_table_;
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

const int64_t IGraph::kUNDEFINED = 1'000'000'000;

std::vector<std::vector<WeightT>> FindShortestPathsBetweenAll(const IGraph& graph) {
    std::vector<std::vector<WeightT>> old_weights = graph.GetEdges();
    std::vector<std::vector<WeightT>> new_weights = old_weights;

    for (size_t i = 1; i < graph.GetVertexesNum() + 1; ++i) {
        std::swap(old_weights, new_weights);
        for (VertexT from = 1; from < static_cast<VertexT>(graph.GetVertexesNum() + 1); ++from) {
            for (VertexT to = 1; to < static_cast<VertexT>(graph.GetVertexesNum() + 1); ++to) {
                new_weights[from][to] = std::min(old_weights[from][to], old_weights[from][i] + old_weights[i][to]);
            }
        }
    }

    return new_weights;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t vertexes_num = 0;

    std::cin >> vertexes_num;
    GraphTable graph(vertexes_num, 0, true);

    for (int32_t i = 1; i < vertexes_num + 1; ++i) {
        for (int32_t j = 1; j < vertexes_num + 1; ++j) {
            WeightT weight = 0;
            std::cin >> weight;
            graph.InsertEdge(i, j, weight);
        }
    }

    std::vector<std::vector<WeightT>> shortest_paths = FindShortestPathsBetweenAll(graph);

    for (int32_t i = 1; i < vertexes_num + 1; ++i) {
        for (int32_t j = 1; j < vertexes_num + 1; ++j) {
            std::cout << shortest_paths[i][j] << ' ';
        }
        std::cout << std::endl;
    }

    return 0;
}