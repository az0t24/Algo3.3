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

    bool IsOriented() const {
        return is_oriented_;
    }

    virtual std::vector<Edge>& GetEdges(const VertexT& vert) = 0;
    virtual const std::vector<Edge>& GetEdges(const VertexT& vert) const = 0;
    virtual std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const = 0;
    virtual std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const = 0;
    virtual std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const = 0;
    virtual void InsertEdge(const VertexT& x, const VertexT& y, const WeightT& weight = 0) = 0;
    virtual void ChangeFlow(Edge edge, WeightT delta) = 0;

protected:
    size_t vertexes_num_;
    size_t edges_num_;
    bool is_oriented_ = false;

private:
    virtual void CreateOppositeEdge(const VertexT source, const VertexT destination) = 0;
};

class GraphList final : public IGraph {
public:
    explicit GraphList(size_t vertexes_num, size_t edges_num = 0, bool is_oriented = false) {
        vertexes_num_ = vertexes_num;
        edges_num_ = edges_num;
        is_oriented_ = is_oriented;
        adjacency_list_.resize(vertexes_num_ + 1, {});
    }

    explicit GraphList(const IGraph& graph) {
        vertexes_num_ = graph.GetVertexesNum();
        is_oriented_ = graph.IsOriented();
        adjacency_list_.resize(vertexes_num_ + 1);

        for (size_t current = 0; current < graph.GetVertexesNum(); ++current) {
            for (auto edge : graph.GetEdges(current)) {
                InsertEdge(edge.from_, edge.to_, edge.weight_);
            }
        }
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

    void ChangeFlow(Edge edge, WeightT delta) override {
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

    void CreateOppositeEdge(const VertexT source, const VertexT destination) override {
        for (auto edge : adjacency_list_[destination]) {
            if (edge.to_ == source) {
                return;
            }
        }
        InsertEdge(destination, source);
    }
};

const int32_t IGraph::kUNDEFINED = INT_MAX;

WeightT FindPath(IGraph& graph, const VertexT source, const VertexT destination, std::vector<IGraph::Edge>& path,
                 std::vector<bool>& used) {
    if (source == destination) {
        return -1;
    }

    if (used[source]) {
        return 0;
    }

    used[source] = true;
    WeightT delta = 0;

    for (auto edge : graph.GetEdges(source)) {
        if (edge.flow_capacity_ - edge.flow_ <= 0) {
            continue;
        }

        if ((delta = FindPath(graph, edge.to_, destination, path, used)) != 0) {
            path.push_back(edge);
            if (delta == -1) {
                return edge.flow_capacity_ - edge.flow_;
            }
            return std::min(delta, edge.flow_capacity_ - edge.flow_);
        }
    }
    return 0;
}

WeightT FindMaxFlowFordFulkerson(const IGraph& graph, const VertexT source, const VertexT destination) {
    WeightT result = 0;
    WeightT additional_flow = 0;
    GraphList temp_graph(graph);

    std::vector<IGraph::Edge> path;
    std::vector<bool> used_normal(graph.GetVertexesNum(), false);
    std::vector<bool> used(graph.GetVertexesNum(), false);

    while ((additional_flow = FindPath(temp_graph, source, destination, path, used)) > 0) {
        for (auto edge : path) {
            temp_graph.ChangeFlow(edge, additional_flow);
        }
        result += additional_flow;

        path.clear();
        used = used_normal;
    }

    return result;
}

int main() {
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

    std::cout << FindMaxFlowFordFulkerson(graph, 1, vertexes_num) << std::endl;

    return 0;
}