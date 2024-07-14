#include "../vertex.hpp"
#include <algorithm>

struct edge {
    int a, b, w;
};

std::vector<edge> get_edge_list(std::vector<Vertex*> g) {
    std::vector<edge> r;
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            edge e;
            e.a = g[i]->val();
            e.b = g[i]->adj(j)->val();
            e.w = g[i]->weight(j);
            r.push_back(e);
        }
    }
    std::sort(r.begin(), r.end(), [](const auto &a, const auto &b) {
        return a.w < b.w;
    });
    return r;
}