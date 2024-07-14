#include "../vertex.hpp"
#include <queue>

int prim(std::vector<Vertex*> g) {
    int total = 0;
    int visitedAmount = 0;
    std::vector<bool> visited(g.size(), false);
    std::priority_queue<std::pair<int, Vertex*>, std::vector<std::pair<int, Vertex*> >, std::greater<std::pair<int, Vertex*> > > pq;
    pq.emplace(0, g[0]);
    while (visitedAmount < g.size()) {
        Vertex* current = pq.top().second;
        visited[current->val()] = true;
        total += pq.top().first;
        visitedAmount++;
        pq.pop();
        int l = current->len();
        for (int i = 0; i<l; i++) {
            if (!visited[current->adj(i)->val()]) pq.emplace(current->weight(i), current->adj(i));
        }
    }
    return total;
}