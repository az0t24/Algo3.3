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

        Edge(const VertexT& first, const VertexT& second, const VertexT& weight) {
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

const int32_t IGraph::kUNDEFINED = INT_MAX;

WeightT PrimMstFindArray(const IGraph& graph) {
    std::vector<DistT> dist(graph.GetVertexesNum() + 1, IGraph::kUNDEFINED);
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

WeightT PrimMstFindHeap(const IGraph& graph) {
    std::vector<DistT> dist(graph.GetVertexesNum() + 1, IGraph::kUNDEFINED);
    std::vector<VertexT> prev(graph.GetVertexesNum() + 1, -1);
    std::set<IGraph::GraphNeighboursNode> mst;
    dist[1] = 0;
    for (VertexT i = 1; i < static_cast<VertexT>(graph.GetVertexesNum() + 1); ++i) {
        mst.insert({i, dist[i]});
    }
    WeightT answer = 0;

    while (!mst.empty()) {
        VertexT current_vertex = mst.begin()->vertex_;
        mst.erase(mst.begin());

        if (prev[current_vertex] != -1) {
            answer += dist[current_vertex];
        }

        for (auto current_neighbour : graph.GetNeighbours(current_vertex)) {
            VertexT vertex_to = current_neighbour.vertex_;
            WeightT weight = current_neighbour.weight_;
            if (mst.find({vertex_to, dist[vertex_to]}) != mst.end() && weight < dist[vertex_to]) {
                mst.erase({vertex_to, dist[vertex_to]});
                dist[vertex_to] = weight;
                prev[vertex_to] = current_vertex;
                mst.insert({vertex_to, dist[vertex_to]});
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
    int32_t edges_num = 0;
    std::cin >> vertexes_num >> edges_num;

    GraphList graph(vertexes_num, edges_num);

    for (int32_t i = 0; i < edges_num; ++i) {
        VertexT first = 0;
        VertexT second = 0;
        WeightT value = 0;
        std::cin >> first >> second >> value;
        graph.InsertEdge(first, second, value);
    }

    std::cout << PrimMstFindHeap(graph) << std::endl;

    return 0;
}