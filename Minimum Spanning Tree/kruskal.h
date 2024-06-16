#include "../DSU/dsu.h"
#include <algorithm>

struct edge {
    int a, b, w;
};

std::vector<edge> getEdgeList(std::vector<Vertex*> g) {
    std::vector<edge> r;
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            edge e;
            e.a = g[i]->val()-1;
            e.b = g[i]->adj(j)->val()-1;
            e.w = g[i]->weight(j);
            r.push_back(e);
        }
    }
    std::sort(r.begin(), r.end(), [](const auto &a, const auto &b) {
        return a.w < b.w;
    });
    return r;
}

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
    std::vector<edge> edges = getEdgeList(g);
    if (edges.size() == 0) return 0;
    std::vector<int> f;
    for (Vertex* x: g) {
        f.push_back(x->val());
    }
    DSU dsu(f);
    return kruskal(edges, dsu);
}