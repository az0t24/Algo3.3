#include <algorithm>
#include <climits>
#include <iostream>
#include <vector>
#include <queue>
#include <set>

using DistT = int32_t;
using VertexT = int32_t;
using WeightT = int32_t;

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

    std::vector<Edge>& GetEdges(const VertexT& vert) override {
        return adjacency_list_[vert];
    }

    const std::vector<Edge>& GetEdges(const VertexT& vert) const override {
        return adjacency_list_[vert];
    }

    void ChangeFlow(Edge edge, WeightT delta) {
        int32_t i = 0;
        while (edge.to_ != adjacency_list_[edge.from_][i].to_ || edge.from_ != adjacency_list_[edge.from_][i].from_) {
            ++i;
        }
        adjacency_list_[edge.from_][i].flow_ += delta;
        CreateOppositeEdge(edge.from_, edge.to_);
        i = 0;
        while (edge.to_ != adjacency_list_[edge.to_][i].from_ || edge.from_ != adjacency_list_[edge.to_][i].to_) {
            ++i;
        }
        adjacency_list_[edge.to_][i].flow_ -= delta;
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

    void CreateOppositeEdge(const VertexT source, const VertexT destination) {
        for (auto edge : adjacency_list_[destination]) {
            if (edge.to_ == source) {
                return;
            }
        }
        InsertEdge(destination, source);
    }
};

const int32_t IGraph::kUNDEFINED = INT_MAX;

void FindShortestPathToAllVertexes(const IGraph& graph, const VertexT vertex, std::vector<WeightT>& dist) {
    dist.resize(graph.GetVertexesNum() + 1, 1'000'000);
    dist[vertex] = 0;
    std::queue<VertexT> temp_queue;
    std::vector<bool> is_in_queue(graph.GetVertexesNum() + 1, false);
    temp_queue.push(vertex);

    while (!temp_queue.empty()) {
        VertexT current_vertex = temp_queue.front();
        temp_queue.pop();
        is_in_queue[current_vertex] = false;

        for (auto edge : graph.GetEdges(current_vertex)) {
            if (dist[edge.to_] > dist[current_vertex] + edge.weight_ && !is_in_queue[edge.to_]) {
                dist[edge.to_] = dist[current_vertex] + edge.weight_;
                temp_queue.push(edge.to_);
            }
        }
    }
}

WeightT FindDjikstra(const IGraph& graph, const std::vector<WeightT>& potentials, const VertexT start) {
    std::vector<WeightT> dist(graph.GetVertexesNum() + 1, 1'000'000);
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
            if (dist[vertex_to] > dist[current_vertex] + weight + potentials[current_vertex] - potentials[vertex_to]) {
                dist[vertex_to] = dist[current_vertex] + weight + potentials[current_vertex] - potentials[vertex_to];
                heap.push({vertex_to, -dist[vertex_to]});
            }
        }
    }

    WeightT result = 0;

    for (VertexT i = 0; i < static_cast<VertexT>(graph.GetVertexesNum()); ++i) {
        if (dist[i] - potentials[start] + potentials[i] > result && dist[i] != 1'000'000) {
            result = dist[i] - potentials[start] + potentials[i];
        }
    }

    return result;
}

WeightT FindMinPathsBetweenAllVertexesJohnson(const IGraph& graph) {
    std::vector<WeightT> potentials;
    FindShortestPathToAllVertexes(graph, graph.GetVertexesNum(), potentials);

    WeightT result = 0;
    WeightT max = 0;

    for (VertexT i = 0; i < static_cast<VertexT>(graph.GetVertexesNum()); ++i) {
        result = FindDjikstra(graph, potentials, i);
        if (result > max) {
            max = result;
        }
    }

    return max;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t vertexes_num = 0;
    int32_t edges_num = 0;
    std::cin >> vertexes_num >> edges_num;

    GraphList graph(vertexes_num, edges_num, true);

    for (int32_t i = 0; i < edges_num; ++i) {
        VertexT from = 0;
        VertexT to = 0;
        WeightT weight = 0;
        std::cin >> from >> to >> weight;

        graph.InsertEdge(from, to, weight);
    }

    for (int32_t i = 0; i < vertexes_num; ++i) {
        graph.InsertEdge(vertexes_num, i, 0);
    }

    std::cout << FindMinPathsBetweenAllVertexesJohnson(graph) << std::endl;

    return 0;
}