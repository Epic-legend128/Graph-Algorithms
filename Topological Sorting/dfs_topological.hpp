#include "../vertex.hpp"
#include <stack>
#include <iostream>

void dfs_topo(Vertex* v, std::stack<Vertex*>& s, std::unordered_map<Vertex*, bool>& visited) {
    visited[v] = true;
    int len = v->len();
    for (int i = 0; i<len; i++) {
        if (visited.find(v->adj(i)) == visited.end()) {
            dfs_topo(v->adj(i), s, visited);
        }
    }
    s.push(v);
}

std::vector<Vertex*> dfs_topological(std::vector<Vertex*>& g) {
    std::stack<Vertex*> s;
    std::unordered_map<Vertex*, bool> visited;
    for (int i = 0; i<g.size(); i++) {
        if (visited.find(g[i]) == visited.end()) {
            dfs_topo(g[i], s, visited);
        }
    }
    std::vector<Vertex*> r;
    for (int i = 0; i<g.size(); i++) {
        r.push_back(s.top());
        s.pop();
    }
    return r;
}