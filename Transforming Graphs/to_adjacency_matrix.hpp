#include "../vertex.hpp"
#include <limits>

std::vector<std::vector<int> > get_adjacency_matrix(std::vector<Vertex*> g) {
    std::vector<int> temp(g.size(), std::numeric_limits<int>::max());
    std::vector<std::vector<int> > r(g.size(), temp);
    for (int i = 0; i<g.size(); i++) {
        r[i][i] = 0;
        for (int j = 0; j<g[i]->len(); j++) {
            r[i][g[i]->adj(j)->val()] = g[i]->weight(j);
        }
    }
    
    return r;
}