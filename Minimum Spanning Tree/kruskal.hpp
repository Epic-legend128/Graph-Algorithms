#include "../vertex.hpp"
#include "../DSU/dsu.hpp"
#include "../Transforming Graphs/to_edge_list.hpp"
#include <algorithm>

int kruskal(std::vector<edge>& edges, DSU dsu) {
    if (edges.size() == 0) return 0;
    int total = 0;
    for (edge e: edges) {
        if (dsu.find(e.a) != dsu.find(e.b)) {
            dsu.unite(e.a, e.b);
            total += e.w;
        }
    }
    return total;
}

int kruskal(std::vector<Vertex*>& g) {
    std::vector<edge> edges = get_edge_list(g);
    
    if (edges.size() == 0) return 0;
    std::vector<int> f;
    for (Vertex* x: g) {
        f.push_back(x->val());
    }
    DSU dsu(f);
    return kruskal(edges, dsu);
}