#include "../vertex.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>

Vertex* bfs(Vertex *head, int key) {
    std::queue<Vertex*> q;
    q.push(head);
    std::unordered_map<Vertex*, bool> visited;
    while (!q.empty()) {
        Vertex* current = q.front();
        if (current->val() == key) return current;
        q.pop();
        if (visited[current]) continue;
        visited[current] = true;
        int l = current->len();
        for (int i = 0; i < l; i++) {
            Vertex* temp = current->adj(i);
            if (visited.find(temp) == visited.end()) {
                q.push(temp);
            }
        }
    }
    return nullptr;
}
