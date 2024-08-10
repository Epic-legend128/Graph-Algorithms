#include "../vertex.hpp"
#include "../DSU/dsu.hpp"
#include "../Transforming Graphs/get_edge_list.hpp"
#include <algorithm>

int kruskal(std::vector<edge>& edges) {
    if (edges.size() == 0) return 0;
    
    std::sort(edges.begin(), edges.end(), [](const auto &a, const auto &b) {
        return a.w < b.w;
    });

    std::vector<int> f;
    for (int i = 0; i<edges.size(); i++) {
        f.push_back(i);
    }
    DSU dsu(f);
    
    int total = 0;
    for (edge e: edges) {
        if (dsu.find(e.a) != dsu.find(e.b)) {
            dsu.unite(e.a, e.b);
            total += e.w;
        }
    }
    return total;
}