#include "../vertex.hpp"
#include <queue>
#include <iostream>

std::vector<Vertex*> kahn(std::vector<Vertex*>& g) {
    std::unordered_map<Vertex*, int> in;
    for (int i = 0; i<g.size(); i++) {
        int l = g[i]->len();
        for (int j = 0; j<l; j++) {
            if (in.find(g[i]->adj(j)) == in.end()) in[g[i]->adj(j)] = 1;
            else in[g[i]->adj(j)]++;
        }
    }
    
    std::queue<Vertex*> q;
    for (int i = 0; i<g.size(); i++) {
        if (in.find(g[i]) == in.end()) q.push(g[i]);
    }

    std::vector<Vertex*> r;
    while (!q.empty()) {
        Vertex* v = q.front();
        q.pop();
        r.push_back(v);
        int l = v->len();
        for (int i = 0; i<l; i++) {
            if ((--in[v->adj(i)]) == 0) q.push(v->adj(i));
        }
    }
    if (r.size() != g.size()) {
        return {};
    }
    return r;
}