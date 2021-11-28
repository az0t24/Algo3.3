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

const int64_t IGraph::kUNDEFINED = 1'000'000'000;

bool FindShortestPathsBetweenAll(const IGraph& graph, std::vector<VertexT>& path) {
    std::vector<WeightT> dist(graph.GetVertexesNum() + 1, 1'000'000);
    std::vector<VertexT> prev(graph.GetVertexesNum() + 1, -1);

    VertexT last = -1;

    for (size_t i = 0; i < graph.GetVertexesNum(); ++i) {
        last = -1;
        for (size_t current_vertex = 1; current_vertex < graph.GetVertexesNum() + 1; ++current_vertex) {
            for (auto edge : graph.GetEdges(current_vertex)) {
                if (dist[edge.to_] > dist[current_vertex] + edge.weight_) {
                    //                    std::cout << edge.to_ << std::endl;
                    dist[edge.to_] = dist[current_vertex] + edge.weight_;
                    prev[edge.to_] = current_vertex;
                    last = edge.to_;
                }
            }
        }
    }

    if (last == -1) {
        return false;
    }

    VertexT first = prev[last];
    for (size_t i = 0; i < graph.GetVertexesNum(); ++i) {
        first = prev[first];
    }

    for (VertexT current = first;; current = prev[current]) {
        path.push_back(current);
        if (current == first && path.size() > 1) {
            break;
        }
    }

    std::reverse(path.begin(), path.end());
    return true;
    //
    //    std::queue<VertexT> temp_queue;
    //    temp_queue.push(last);
    //    bool check = true;
    //
    //    while (!temp_queue.empty() && check) {
    //        VertexT current_vertex = temp_queue.front();
    //        temp_queue.pop();
    //
    //        for (auto edge : graph.GetEdges(current_vertex)) {
    //                        std::cout << dist[edge.to_] << ' ' << dist[current_vertex] << std::endl;
    //            if (dist[edge.to_] == dist[current_vertex] + edge.weight_) {
    //                                std::cout << edge.to_ << std::endl;
    //                dist[edge.to_] = dist[current_vertex] + edge.weight_;
    //                prev[edge.to_] = current_vertex;
    //                if (edge.to_ == last) {
    //                    //                    std::cout << "here";
    //                    check = false;
    //                    break;
    //                }
    //                temp_queue.push(edge.to_);
    //            }
    //        }
    //    }
    //
    //    path.push_back(last);
    //    VertexT current = prev[last];
    //    while (last != current) {
    //        path.push_back(current);
    //        current = prev[current];
    //        if (current == -1 || current == 0) {
    //            return false;
    //        }
    //    }
    //    path.push_back(current);
    //
    //    std::reverse(path.begin(), path.end());
    //
    //    return true;
}

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    int32_t vertexes_num = 0;

    std::cin >> vertexes_num;
    GraphList graph(vertexes_num, 0, true);

    for (int32_t i = 1; i < vertexes_num + 1; ++i) {
        for (int32_t j = 1; j < vertexes_num + 1; ++j) {
            WeightT weight = 0;

            std::cin >> weight;
            if (weight == 100'000) {
                continue;
            }

            graph.InsertEdge(i, j, weight);
        }
    }

    std::vector<VertexT> negative_cycle;

    if (!FindShortestPathsBetweenAll(graph, negative_cycle)) {
        std::cout << "NO" << std::endl;
    } else {
        std::cout << "YES" << std::endl;
        std::cout << negative_cycle.size() << std::endl;
        for (auto& vertex : negative_cycle) {
            std::cout << vertex << ' ';
        }
        std::cout << std::endl;
    }

    return 0;
}