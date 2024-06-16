#include <iostream>
#include "vertex.h"
#include "Searching/bfs.h"
#include "Searching/dfs.h"
#include "Searching/dfs_recursive.h"
#include "Shortest Path/dijkstra.h"
#include "Shortest Path/bellman_ford.h"
#include "Shortest Path/floyd_warshall.h"
#include "Negative Cycle/has_negative_cycle.h"
#include "Minimum Spanning Tree/kruskal.h"

int main() {
    std::vector<Vertex*> g;
    Vertex p1;
    Vertex p2;
    Vertex p3;
    Vertex p4;
    Vertex p5;
    p1.val(1);
    p2.val(2);
    p3.val(3);
    p4.val(4);
    p5.val(5);
    p1.add(&p2, 7);
    p2.add(&p4, 3);
    p3.add(&p2, 2);
    p4.add(&p5, 2);
    p1.add(&p3, 3);
    p5.add(&p2, 3);
    g.push_back(&p1);
    g.push_back(&p2);
    g.push_back(&p3);
    g.push_back(&p4);
    g.push_back(&p5);
    
    Vertex* r = dfs_recursive(&p3, p5.val()); // substitute with bfs or dfs
    if (r == nullptr) std::cout << "nullptr\n";
    else std::cout << r->val() <<" with amount of neighbours being "<<r->len()<< '\n';

    int r2 = dijkstra(&p1, p2.val());
    if (r2 == -1) std::cout << "Not found\n";
    else std::cout << "The shortest path is of weight "<<r2<< '\n';

    int start = 0;
    std::vector<int> shortest = bellman_ford(g, start);
    std::cout << "Starting from "<< (start+1) << std::endl;
    for (int i = 0; i<shortest.size(); i++) {
        std::cout << "Shortest path for "<<(i+1)<<" is: "<<shortest[i]<<std::endl;
    }
    
    std::cout << "Does the graph have a negative cycle between the nodes that can be accessed from node "<<(start+1)<<"? "<< (has_negative_cycle(g, start) ? "Yes" : "No")<< '\n';


    std::vector<std::vector<int> > dists;
    floyd_warshall(g, dists);
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g.size(); j++) {
            std::cout << "The distance from "<<(i+1)<<" to "<<(j+1)<<" is "<<dists[i][j]<<std::endl;
        }
    }

    int cost = kruskal(g);
    std::cout << "Minimum Spanning Tree Cost: "<<cost<<'\n';
    return 0;
}

/* 
Graph
1
2
3
4
5
1 2 7
2 4 3
3 2 2
4 5 2
1 3 3
5 2 3
*/