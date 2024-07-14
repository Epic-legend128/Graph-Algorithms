#include "../vertex.hpp"

std::vector<std::vector<std::pair<bool, int> > > get_adjacency_matrix(std::vector<Vertex*> g) {
    std::vector<std::pair<bool, int> > temp(g.size(), std::make_pair(0, 0));
    std::vector<std::vector<std::pair<bool, int> > > r(g.size(), temp);
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            r[i][g[i]->adj(j)->val()] = std::make_pair(true, g[i]->weight(j));
        }
    }
    
    return r;
}