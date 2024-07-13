#include "../vertex.hpp"
#include <iostream>
#include <vector>
#include <limits>

void floyd_warshall(const std::vector<Vertex*>& arr, std::vector<std::vector<int> >& dists) {
    const int defaultMax = std::numeric_limits<int>::max();
    dists.resize(arr.size());
    for (int i = 0; i<arr.size(); i++) {
        dists[i].resize(arr.size());
        for (int j = 0; j<arr.size(); j++) {
            dists[i][j] = defaultMax;
        }
    }

    for (int i = 0; i<arr.size(); i++) {
        dists[i][i] = 0;
        int l = arr[i]->len();
        for (int j = 0; j < l; j++) {
            int v = arr[i]->adj(j)->val()-1;
            if (dists[i][v] > arr[i]->weight(j)) {
                dists[i][v] = arr[i]->weight(j);
            }
        }
    }

    for (int k = 0; k<arr.size(); k++) {
        for (int i = 0; i<arr.size(); i++) {
            for (int j = 0; j<arr.size(); j++) {
                if (dists[i][k] != defaultMax && dists[k][j] != defaultMax && dists[i][j] > dists[i][k] + dists[k][j]) {
                    dists[i][j] = dists[i][k] + dists[k][j];
                }
            }
        }
    }
}
