#include "../vertex.hpp"
#include <algorithm>

#ifndef EDGE
#define EDGE

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
    return r;
}
#endif