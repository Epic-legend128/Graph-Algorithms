#include <iostream>
#include "vertex.hpp"
#include "Traversal/bfs.hpp"
#include "Traversal/dfs.hpp"
#include "Traversal/dfs_recursive.hpp"
#include "Shortest Path/dijkstra.hpp"
#include "Shortest Path/bellman_ford.hpp"
#include "Shortest Path/floyd_warshall.hpp"
#include "Negative Cycle/has_negative_cycle.hpp"
#include "Minimum Spanning Tree/kruskal.hpp"
#include "Minimum Spanning Tree/prim.hpp"
#include "Strongly Connected Components/tarjan.hpp"
#include "Topological Sorting/dfs_topological.hpp"
#include "Topological Sorting/kahn.hpp"
#include "Transforming Graphs/to_adjacency_matrix.hpp"
#include "Transforming Graphs/to_edge_list.hpp"

int main() {
    std::vector<Vertex*> g; // Normal
    Vertex p1;
    Vertex p2;
    Vertex p3;
    Vertex p4;
    Vertex p5;
    p1.val(0);
    p2.val(1);
    p3.val(2);
    p4.val(3);
    p5.val(4);
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
    std::vector<Vertex*> g2; // DAG
    Vertex p20;
    Vertex p21;
    Vertex p22;
    Vertex p23;
    Vertex p24;
    Vertex p25;
    p20.val(0);
    p21.val(1);
    p22.val(2);
    p23.val(3);
    p24.val(4);
    p25.val(5);
    p22.add(&p23, 3);
    p23.add(&p21, 2);
    p24.add(&p20, 2);
    p24.add(&p21, 2);
    p25.add(&p22, 3);
    p25.add(&p20, 3);
    g2.push_back(&p20);
    g2.push_back(&p21);
    g2.push_back(&p22);
    g2.push_back(&p23);
    g2.push_back(&p24);
    g2.push_back(&p25);
    
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

    std::vector<std::vector<int> > matrix = get_adjacency_matrix(g);
    std::vector<std::vector<int> > dists = floyd_warshall(matrix);
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g.size(); j++) {
            if (dists[i][j] == std::numeric_limits<int>::max()) std::cout << "No path from "<<(i+1)<<" to "<<(j+1)<< std::endl;
            else std::cout << "The distance from "<<(i+1)<<" to "<<(j+1)<<" is "<<dists[i][j]<< std::endl;
        }
    }

    // kruskal and prim only work on undirected graphs
    std::vector<edge> edges = get_edge_list(g);
    int cost = kruskal(edges); // or use prim(g) instead
    std::cout << "Minimum Spanning Tree Cost: "<<cost<<'\n';

    //tarjan's algorithm for SCCs
    int t = tarjan(g);
    std::cout << "The amount of strongly connected components in the graph is "<<t<<'\n';

    //topological sorting
    std::vector<Vertex*> r_topo = kahn(g2); // can also substitute with dfs_topological(g2)
    if (r_topo.size() == 0) std::cout << "There was a cycle\n"; // only for kahn
    else {
        std::cout << "Topological sorting is: ";
        for (Vertex* v: r_topo) {
            std::cout << v->val()<<" ";
        }
        std::cout << "\n";
    }
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

backup
p1.add(&p2, 1);
p2.add(&p3, 2);
p3.add(&p1, 3);
p4.add(&p5, 4);
p4.add(&p8, 5);
p8.add(&p4, 6);
p8.add(&p6, 7);
p5.add(&p6, 8);
p6.add(&p7, 9);
p6.add(&p1, 10);
p7.add(&p3, 11);
p7.add(&p1, 12);
p7.add(&p5, 12);
g.push_back(&p1);
g.push_back(&p2);
g.push_back(&p3);
g.push_back(&p4);
g.push_back(&p5);
g.push_back(&p6);
g.push_back(&p7);
g.push_back(&p8);
*/