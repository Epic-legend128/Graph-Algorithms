#include <iostream>
#include "vertex.h"
#include "Searching/bfs.h"

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
    int re = p2.valu();
    p1.add(&p2, 1);
    p2.add(&p4, 3);
    p3.add(&p2, 5);
    p4.add(&p5, 2);
    p1.add(&p3, 6);
    p5.add(&p2, 3);
    
    Vertex* r = bfs(&p4, p5.valu());
    if (r == nullptr) std::cout << "nullptr\n";
    else std::cout << r->valu() << '\n';
    return 0;
}

/* 
Graph
1
2
3
4
5
1 2 1
2 4 3
3 2 5
4 5 2
1 3 6
5 2 3
*/