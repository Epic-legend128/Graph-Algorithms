#include <utility>
#include <vector>
#ifndef VERTEX
#define VERTEX

class Vertex {
    private:
        int value;
        std::vector<std::pair<Vertex*, int> > nextVertex;
    public:
        void val(int v) {
            value = v;
        }
        
        int valu() {
            return value;
        }

        void add(Vertex* n, int w) {
            nextVertex.push_back(std::make_pair(n, w)); 
        }

        Vertex* adj(int i) {
            return nextVertex[i].first;
        }

        int len() {
            return nextVertex.size();
        }
};
#endif