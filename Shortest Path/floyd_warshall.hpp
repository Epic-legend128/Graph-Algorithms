#include "../vertex.hpp"
#include <iostream>
#include <vector>
#include <limits>

std::vector<std::vector<int> > floyd_warshall(std::vector<std::vector<int> >& dists) {
    const int defaultMax = std::numeric_limits<int>::max();

    for (int k = 0; k<dists.size(); k++) {
        for (int i = 0; i<dists.size(); i++) {
            for (int j = 0; j<dists.size(); j++) {
                if (dists[i][k] != defaultMax && dists[k][j] != defaultMax && dists[i][j] > dists[i][k] + dists[k][j]) {
                    dists[i][j] = dists[i][k] + dists[k][j];
                }
            }
        }
    }
    return dists;
}