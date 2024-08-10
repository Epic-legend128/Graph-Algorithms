#include "../vertex.hpp"

std::vector<std::vector<std::pair<int, int> > > get_adjacency_list(std::vector<Vertex*> g) {
    std::vector<std::vector<std::pair<int, int> > > r;
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            r[i].emplace_back(g[i]->adj(j)->val(), g[i]->weight(j));
        }
    }
    
    return r;
}