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

    virtual std::vector<Edge>& GetEdges(const VertexT& vert) = 0;
    virtual std::vector<Edge> GetEdges(const VertexT& vert) const = 0;
    virtual std::vector<VertexT> GetNeighboursVertex(const VertexT& vert) const = 0;
    virtual std::vector<WeightT> GetNeighboursWeight(const VertexT& vert) const = 0;
    virtual std::vector<GraphNeighboursNode> GetNeighbours(const VertexT& vert) const = 0;
    virtual void InsertEdge(const VertexT& x, const VertexT& y, const WeightT& weight = 0) = 0;
    virtual void InsertEdge(const VertexT& x, const VertexT& y, const WeightT& time_in, const WeightT& time_out) = 0;

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

    void InsertEdge(const VertexT& first, const VertexT& second, const WeightT& time_in,
                    const WeightT& time_out) override {
        if (is_oriented_) {
            InsertEdgeOriented(first, second, time_in, time_out);
        } else {
            InsertEdgeNotOriented(first, second, time_in, time_out);
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

    std::vector<Edge> GetEdges(const VertexT& vert) const override {
        return adjacency_list_[vert];
    }

private:
    std::vector<std::vector<Edge>> adjacency_list_;  // using adjacency list

    void InsertEdgeOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 0) {
        adjacency_list_[first].emplace_back(first, second, weight);
    }

    void InsertEdgeOriented(const VertexT& first, const VertexT& second, const WeightT& time_in,
                            const WeightT& time_out) {
        adjacency_list_[first].emplace_back(first, second, time_in, time_out);
    }

    void InsertEdgeNotOriented(const VertexT& first, const VertexT& second, const WeightT& weight = 0) {
        adjacency_list_[first].emplace_back(first, second, weight);
        adjacency_list_[second].emplace_back(second, first, weight);
    }

    void InsertEdgeNotOriented(const VertexT& first, const VertexT& second, const WeightT& time_in,
                               const WeightT& time_out) {
        adjacency_list_[first].emplace_back(first, second, time_in, time_out);
        adjacency_list_[second].emplace_back(second, first, time_in, time_out);
    }
};

const int64_t IGraph::kUNDEFINED = 1'000'000'000;

WeightT FindShortestPathsDijkstraArray(const IGraph& graph, const VertexT start, const VertexT end) {
    const int64_t no_path_code = IGraph::kUNDEFINED;
    std::vector<WeightT> dist(graph.GetVertexesNum() + 1, no_path_code);
    std::vector<WeightT> dist_unuse(graph.GetVertexesNum() + 1, no_path_code);

    dist[start] = 0;
    dist_unuse[start] = 0;
    std::set<VertexT> mst = {};

    while (mst.size() != graph.GetVertexesNum()) {
        VertexT current_vertex = std::min_element(dist_unuse.begin(), dist_unuse.end()) - dist_unuse.begin();
        mst.insert(current_vertex);
        dist_unuse[current_vertex] = IGraph::kUNDEFINED + 1;

        for (auto current_neighbour : graph.GetNeighbours(current_vertex)) {
            VertexT vertex_to = current_neighbour.vertex_;
            WeightT weight = current_neighbour.weight_;
            if (mst.find(vertex_to) == mst.end() && dist[vertex_to] > dist[current_vertex] + weight) {
                dist[vertex_to] = weight + dist[current_vertex];
                dist_unuse[vertex_to] = dist[vertex_to];
            }
        }
    }

    return (dist[end] == IGraph::kUNDEFINED) ? -1 : dist[end];
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t vertexes_num = 0;

    std::cin >> vertexes_num;
    GraphList graph(vertexes_num, 0, true);

    VertexT start = 0;
    VertexT end = 0;
    std::cin >> start >> end;

    for (int32_t i = 1; i < vertexes_num + 1; ++i) {
        for (int32_t j = 1; j < vertexes_num + 1; ++j) {
            WeightT weight = 0;

            std::cin >> weight;
            if (weight == 0 || weight == -1) {
                continue;
            }
            graph.InsertEdge(i, j, weight);
        }
    }

    std::cout << FindShortestPathsDijkstraArray(graph, start, end) << std::endl;

    return 0;
}