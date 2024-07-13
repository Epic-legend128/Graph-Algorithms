#include "../vertex.hpp"
#include <unordered_map>
#include <iostream>

Vertex* dfs_recursive(Vertex *head, int key, std::unordered_map<Vertex*, bool>& visited) {
    if (head->val() == key) return head;
    if (visited[head]) return nullptr;
    visited[head] = true;
    int l = head->len();
    for (int i = 0; i < l; i++) {
        Vertex* temp = head->adj(i);
        if (visited.find(temp) == visited.end()) {
            Vertex* r = dfs_recursive(temp, key, visited);
            if (r != nullptr) return r;
        }
    }
    return nullptr;
}

Vertex* dfs_recursive(Vertex *head, int key) {
    std::unordered_map<Vertex*, bool> visited;
    if (head->val() == key) return head;
    if (visited[head]) return nullptr;
    visited[head] = true;
    int l = head->len();
    for (int i = 0; i < l; i++) {
        Vertex* temp = head->adj(i);
        if (visited.find(temp) == visited.end()) {
            Vertex* r = dfs_recursive(temp, key, visited);
            if (r != nullptr) return r;
        }
    }
    return nullptr;
}
