#include "../vertex.hpp"

std::vector<std::vector<int> > get_adjacency_list(std::vector<Vertex*> g) {
    std::vector<std::vector<int > > r;
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            r[i].push_back(g[i]->adj(j)->val());
        }
    }
    
    return r;
}