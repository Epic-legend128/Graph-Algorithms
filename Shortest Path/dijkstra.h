#include "../vertex.h"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <vector>

int dijkstra(Vertex *head, int key) {
    std::priority_queue<std::pair<int, Vertex*>, std::vector<std::pair<int, Vertex*> >, std::greater<std::pair<int, Vertex*> > > q;
    q.push(std::make_pair(0, head));
    std::unordered_map<Vertex*, int> weights;
    while (!q.empty()) {
        Vertex* current = q.top().second;
        int weight = q.top().first;
        if (current->val() == key) return weight;
        q.pop();
        weights[current] = weight;
        int l = current->len();
        for (int i = 0; i < l; i++) {
            Vertex* temp = current->adj(i);
            int w = current->weight(i);
            if (weights.find(temp) == weights.end() || weight+w < weights[temp]) {
                weights[temp] = weight+w;
                q.push(std::make_pair(weight+w, temp));
            }
        }
    }
    return -1;
}
