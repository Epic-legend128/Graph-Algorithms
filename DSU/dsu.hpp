#include <vector>
class DSU {
    private:
        std::vector<int> parents;
        std::vector<int> ranks;
    public:
        DSU(std::vector<int>& p) {
            parents.resize(p.size());
            ranks.resize(p.size());
            for (int x: p) {
                parents[x] = x;
                ranks[x] = 0;
            }
        }

        int find(int i) {
            if (parents[i] != i) parents[i] = find(parents[i]);
            return parents[i];
        }

        void unite(int i, int j) {
            int first = find(i);
            int second = find(j);
            if (i == j) return;
            if (ranks[first] > ranks[second]) {
                parents[second] = first;
            }
            else {
                parents[first] = second;
                if (ranks[first] == ranks[second]) {
                    ranks[second]++;
                }
            }
        }
};