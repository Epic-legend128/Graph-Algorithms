#include <iostream>
#include "vertex.h"
#include "Searching/bfs.h"
#include "Searching/dfs.h"
#include "Searching/dfs_recursive.h"
#include "Shortest Path/dijkstra.h"

int main() {
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
    
    Vertex* r = dfs_recursive(&p3, p5.val()); // substitute with bfs or dfs
    if (r == nullptr) std::cout << "nullptr\n";
    else std::cout << r->val() <<" with amount of neighbours being "<<r->len()<< '\n';

    int r2 = dijkstra(&p1, p2.val());
    if (r2 == -1) std::cout << "Not found\n";
    else std::cout << "The shortest path is of weight "<<r2<< '\n';
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