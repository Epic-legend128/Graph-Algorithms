#include "../vertex.hpp"
#include <vector>
#include <limits>
#include <queue>
#include <iostream>

bool has_negative_cycle(std::vector<Vertex*>& arr, int start) {
    const int defaultWeight = std::numeric_limits<int>::max();
    std::vector<int> total(arr.size(), defaultWeight);
    total[start] = 0;
    int counter = 0;
    while (true && counter < arr.size()-1) {
        bool changed = false;
        for (int i = 0; i<arr.size(); i++) {
            if (total[i] == defaultWeight) continue;
            int l = arr[i]->len();
            for (int j = 0; j<l; j++) {
                int v = arr[i]->adj(j)->val();
                if (total[i] + arr[i]->weight(j) < total[v]) {
                    changed = true;
                    total[v] = total[i] + arr[i]->weight(j);
                }
            }
        }
        if (!changed) break;
        counter++;
    }

    bool changed = false;
    for (int i = 0; i<arr.size(); i++) {
        if (total[i] == defaultWeight) continue;
        int l = arr[i]->len();
        for (int j = 0; j<l; j++) {
            int v = arr[i]->adj(j)->val();
            if (total[i] + arr[i]->weight(j) < total[v]) {
                changed = true;
                total[v] = total[i] + arr[i]->weight(j);
            }
        }
    }
    return changed;
}

bool has_negative_cycle(std::vector<Vertex*>& arr) {
    return has_negative_cycle(arr, 0);
}