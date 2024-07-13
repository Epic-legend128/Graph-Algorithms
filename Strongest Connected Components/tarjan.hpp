#include "../vertex.hpp"
#include <vector>
#include <stack>
#include <unordered_map>
#include <iostream>

void dfs_t(Vertex* head, std::unordered_map<Vertex*, int>& lows, std::unordered_map<Vertex*, int>& ids, int& amount, std::unordered_map<Vertex*, bool>& onStack, std::stack<Vertex*>& s, int& id) {
    s.push(head);
    onStack[head] = true;
    lows[head] = id;
    ids[head] = id++;
    int len = head->len();
    for (int i = 0; i<len; i++) {
        Vertex* temp = head->adj(i);
        if (ids.find(temp) == ids.end()) dfs_t(temp, lows, ids, amount, onStack, s, id);
        if (onStack[temp] && (lows.find(head) == lows.end() || lows[head] > lows[temp])) lows[head] = lows[temp];
    }

    if (ids[head] == lows[head]) {
        for (Vertex* node = s.top(); true; node = s.top()) {
            s.pop();
            onStack[node] = false;
            lows[node] = lows[head];
            if (node == head) break;
        }
        amount++;
    }
}

int tarjan(std::vector<Vertex*>& g) {
    int n = g.size();
    int amount = 0;
    int id = 0;
    std::unordered_map<Vertex*, int> ids;
    std::unordered_map<Vertex*, int> lows;
    std::unordered_map<Vertex*, bool> onStack;
    std::stack<Vertex*> s;
    for (int i = 0; i<n; i++) {
        if (ids.find(g[i]) == ids.end()) dfs_t(g[i], lows, ids, amount, onStack, s, id);
    }
    return amount;
}