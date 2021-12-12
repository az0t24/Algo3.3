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

template <typename Data>  // Data with info about edge
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
        Data data_;
        int32_t number_;

        Edge(const VertexT& first, const VertexT& second, const VertexT& weight, const Data& data) {
            from_ = first;
            to_ = second;
            weight_ = weight;
            data_ = data;
            number_ = 0;
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

    bool IsOriented() const {
        return is_oriented_;
    }

    virtual std::vector<Edge>& GetEdges(const VertexT& vert) = 0;
    virtual const std::vector<Edge>& GetEdges(const VertexT& vert) const = 0;
    virtual std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const = 0;
    virtual std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const = 0;
    virtual std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const = 0;
    virtual void InsertEdge(const VertexT& x, const VertexT& y, const WeightT& weight = 0, const Data& data = 0) = 0;
    virtual void ChangeEdge(const Edge& edge, const WeightT& delta) = 0;
    virtual Edge GetOppositeEdge(const VertexT source, const VertexT destination) = 0;

protected:
    size_t vertexes_num_;
    size_t edges_num_;
    bool is_oriented_ = false;
};

template <typename Data>
class GraphList final : public IGraph<Data> {
public:
    explicit GraphList(size_t vertexes_num, size_t edges_num = 0, bool is_oriented = false) {
        this->vertexes_num_ = vertexes_num;
        this->edges_num_ = edges_num;
        this->is_oriented_ = is_oriented;
        adjacency_list_.resize(this->vertexes_num_ + 1, {});
    }

    explicit GraphList(const IGraph<Data>& graph) {
        this->vertexes_num_ = graph.GetVertexesNum();
        this->is_oriented_ = graph.IsOriented();
        adjacency_list_.resize(this->vertexes_num_ + 1);

        for (size_t current = 0; current < graph.GetVertexesNum(); ++current) {
            for (auto edge : graph.GetEdges(current)) {
                InsertEdge(edge.from_, edge.to_, edge.weight_);
            }
        }
    }

    void InsertEdge(const VertexT& first, const VertexT& second, const WeightT& weight = 0,
                    const Data& data = 0) override {
        if (this->is_oriented_) {
            InsertEdgeOriented(first, second, weight, data);
        } else {
            InsertEdgeNotOriented(first, second, weight, data);
        }
    }

    std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const override {
        std::vector<VertexT> neighbours_vertexes(adjacency_list_[vert].size());
        for (size_t i = 0; i < neighbours_vertexes.size(); ++i) {
            neighbours_vertexes[i] = adjacency_list_[vert][i].to_;
        }
        return neighbours_vertexes;
    }

    std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const override {
        std::vector<WeightT> neighbours_weights(adjacency_list_[vert].size());
        for (size_t i = 0; i < neighbours_weights.size(); ++i) {
            neighbours_weights[i] = adjacency_list_[vert][i].weight_;
        }
        return neighbours_weights;
    }

    std::vector<typename IGraph<Data>::GraphNeighboursNode> GetNeighbours(const VertexT& vert) const override {
        std::vector<typename IGraph<Data>::GraphNeighboursNode> neighbours;
        for (auto edge : adjacency_list_[vert]) {
            neighbours.push_back({edge.to_, edge.weight_});
        }
        return neighbours;
    }

    std::vector<typename IGraph<Data>::Edge>& GetEdges(const VertexT& vert) override {
        return adjacency_list_[vert];
    }

    const std::vector<typename IGraph<Data>::Edge>& GetEdges(const VertexT& vert) const override {
        return adjacency_list_[vert];
    }

    void ChangeEdge(const typename IGraph<Data>::Edge& edge, const WeightT& delta) override {
        int32_t i = 0;
        while (edge.to_ != adjacency_list_[edge.from_][i].to_ || edge.from_ != adjacency_list_[edge.from_][i].from_) {
            ++i;
        }
        adjacency_list_[edge.from_][i].weight_ += delta;
    }

    typename IGraph<Data>::Edge GetOppositeEdge(const VertexT source, const VertexT destination) override {
        for (auto edge : adjacency_list_[destination]) {
            if (edge.to_ == source) {
                return edge;
            }
        }
        InsertEdge(destination, source);
        return {destination, source, 0, 0};
    }

private:
    std::vector<std::vector<typename IGraph<Data>::Edge>> adjacency_list_;  // using adjacency list

    void InsertEdgeOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 0,
                            const Data& data = 0) {
        adjacency_list_[first].emplace_back(first, second, weight, data);
    }

    void InsertEdgeNotOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 0,
                               const Data& data = 0) {
        adjacency_list_[first].emplace_back(first, second, weight, data);
        adjacency_list_[second].emplace_back(second, first, weight, data);
    }
};

template <typename Data>
const int32_t IGraph<Data>::kUNDEFINED = 1'000'000'000;

WeightT FindPathLength(const IGraph<WeightT>& graph, const VertexT from, const VertexT to) {
    std::vector<WeightT> dist(graph.GetVertexesNum() + 1, IGraph<WeightT>::kUNDEFINED);

    dist[from] = 0;

    for (size_t i = 0; i < graph.GetEdgesNum(); ++i) {
        for (VertexT current_vertex = 1; current_vertex < static_cast<VertexT>(graph.GetVertexesNum() + 1);
             ++current_vertex) {
            for (auto edge : graph.GetEdges(current_vertex)) {
                if (dist[edge.from_] <= edge.data_ && edge.data_ - edge.weight_ < dist[edge.to_]) {
                    dist[edge.to_] = edge.data_ - edge.weight_;
                }
            }
        }
    }

    return dist[to];
}

int main() {
    int32_t stations = 0;
    VertexT start = 0;
    VertexT end = 0;
    int32_t teleports = 0;
    std::cin >> stations >> start >> end >> teleports;

    GraphList<WeightT> graph(stations, teleports, true);

    for (int32_t i = 0; i < teleports; ++i) {
        VertexT from = 0;
        VertexT to = 0;
        WeightT time_in = 0;
        WeightT time_out = 0;
        std::cin >> from >> time_out >> to >> time_in;

        graph.InsertEdge(from, to, time_out - time_in, time_out);
    }

    std::cout << FindPathLength(graph, start, end) << std::endl;

    return 0;
}