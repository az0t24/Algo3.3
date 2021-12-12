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
        int32_t number_;

        Edge(const VertexT& first, const VertexT& second, const VertexT& weight) {
            from_ = first;
            to_ = second;
            weight_ = weight;
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

    static const int64_t kUNDEFINED;

    size_t GetVertexesNum() const {
        return vertexes_num_;
    }

    size_t GetEdgesNum() const {
        return edges_num_;
    }

    virtual std::vector<Edge>& GetEdges(const VertexT& vert) = 0;
    virtual const std::vector<Edge>& GetEdges(const VertexT& vert) const = 0;
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

    void Resize(size_t new_size) {
        adjacency_list_.resize(new_size + 1, {});
        vertexes_num_ = new_size;
    }

    void InsertEdge(const VertexT& first, const VertexT& second, const WeightT& weight = 0) override {
        if (is_oriented_) {
            InsertEdgeOriented(first, second, weight);
        } else {
            InsertEdgeNotOriented(first, second, weight);
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

    std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const override {
        std::vector<GraphNeighboursNode> neighbours;
        for (auto edge : adjacency_list_[vert]) {
            neighbours.push_back({edge.to_, edge.weight_});
        }
        return neighbours;
    }

    std::vector<Edge>& GetEdges(const VertexT& vert) override {
        return adjacency_list_[vert];
    }

    const std::vector<Edge>& GetEdges(const VertexT& vert) const override {
        return adjacency_list_[vert];
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

const int64_t IGraph::kUNDEFINED = 1'000'000'000;

WeightT FindShortestPath(const IGraph& graph, const VertexT start, const VertexT end) {
    std::vector<WeightT> dist(graph.GetVertexesNum() + 1, IGraph::kUNDEFINED);
    std::priority_queue<IGraph::GraphNeighboursNode> heap;
    dist[start] = 0;
    heap.push({start, 0});

    while (!heap.empty()) {
        VertexT current_vertex = heap.top().vertex_;
        WeightT current_weight = -heap.top().weight_;
        heap.pop();

        if (current_weight > dist[current_vertex]) {
            continue;
        }

        for (auto& current_neighbour : graph.GetEdges(current_vertex)) {
            VertexT vertex_to = current_neighbour.to_;
            WeightT weight = current_neighbour.weight_;
            if (dist[vertex_to] > dist[current_vertex] + weight) {
                dist[vertex_to] = weight + dist[current_vertex];
                heap.push({vertex_to, -dist[vertex_to]});
            }
        }
    }

    return dist[end];
}

void FillGraph(IGraph& graph, const std::vector<VertexT>& stops, const std::vector<int32_t>& lifts_stops,
               const VertexT& max_stop, const int32_t lifts_num, const int32_t up, const int32_t down, const int32_t in,
               const int32_t out) {
    int32_t current_stop = 0;
    for (int32_t i = 0; i < lifts_num; ++i) {
        auto j = 0;
        while (j < lifts_stops[i]) {
            graph.InsertEdge(stops[current_stop], max_stop + i, in);
            graph.InsertEdge(max_stop + i, stops[current_stop], out);
            current_stop++;
            j++;
        }
    }

    for (auto i = 1; i < max_stop; ++i) {
        if (i > 1) {
            graph.InsertEdge(i, i - 1, down);
        }
        if (i < max_stop - 1) {
            graph.InsertEdge(i, i + 1, up);
        }
    }
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t number = 0;
    int32_t down = 0;
    int32_t up = 0;
    int32_t in = 0;
    int32_t out = 0;
    int32_t lifts_num = 0;

    std::cin >> number >> up >> down >> in >> out >> lifts_num;

    VertexT max_stop = number;

    std::vector<VertexT> stops;
    std::vector<int32_t> lifts;

    for (int32_t i = 0; i < lifts_num; ++i) {
        size_t stops_num = 0;
        std::cin >> stops_num;
        for (size_t j = 0; j < stops_num; ++j) {
            VertexT stop = 0;
            std::cin >> stop;
            if (stop > max_stop) {
                max_stop = stop;
            }
            stops.push_back(stop);
        }
        lifts.push_back(stops_num);
    }

    GraphList graph(max_stop + lifts_num, 0, true);

    FillGraph(graph, stops, lifts, max_stop + 1, lifts_num, up, down, in, out);

    std::cout << FindShortestPath(graph, 1, number) << std::endl;

    return 0;
}