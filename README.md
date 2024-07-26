# Graph-Algorithms
Explanation and implementation of a bunch of different graph-related algorithms.

## Table of Contents
* [Graph Representation](#graph-representation)
    - [Vector of Vertices](#vector-of-vertices)
    - [Adjacency Matrix](#adjacency-matrix)
    - [Adjacency List](#adjacency-list)
    - [Edge List](#edge-list)
* [Searching](#searching)
    - [Breadth-Frst Search](#breadth-first-search)
    - [Depth-First Search](#depth-first-search)
* [Shortest Path](#shortest-path)
    - [Dijkstra](#dijkstra)
    - [Bellman-Ford](#bellman-ford)
    - [Floyd-Warshall](#floyd-warshall)
* [Topological Sorting](#topological-sorting)
    - [Using DFS](#using-dfs)
    - [Kahn's Algorithm](#kahns-algorithm)
* [Minimum Spanning Tree](#minimum-spanning-tree)
    - [Kruskal](#kruskal)
    - [Prim](#prim)
* [Strongly Connected Components](#strongly-connected-components)
    - [Tarjan's Algorithm](#tarjans-algorithm)


## Graph Representation
  A graph is a structure made up of vertices and edges. Vertices, also known as nodes, are connected through the edges. There are many ways to represent a graph in programming, and choosing the right one depends on your purpose. In the code of this repository, I will be using a vector composed of the nodes of the graph, where each node will be represented using a custom struct. However, there are also other ways to accomplish the same thing, each with its own advantages and disadvantages.

### Vector of Vertices
  A vector of vertices is what I am going to be using throughout this repository. Basically, I created a struct to act as a node which holds its value, a pointer to each neighbouring node as well as the weight of the edge connecting them and a way to add edges from one node to another. The code for the node is the following:
  
```c++
class Vertex {
    private:
        int value;
        std::vector<std::pair<Vertex*, int> > nextVertex;
    public:
        void val(int v) {
            value = v;
        }
        
        int val() {
            return value;
        }

        void add(Vertex* n, int w) {
            nextVertex.emplace_back(n, w);
        }

        Vertex* adj(int i) {
            return nextVertex[i].first;
        }

        int weight(int i) {
            return nextVertex[i].second;
        }

        int len() {
            return nextVertex.size();
        }
};
```

### Adjacency Matrix
  An adjacency matrix is basically a 2D board of size NxN where N is the amount of vertices present in the graph. A number is given to each vertex and the place [i][j] in the board symbolises that there is an edge connecting node i and j. In cases where the weight of all edges is constant then a simple 2D boolean board would be enough, however, in cases with varying weights it is necessary to replace booleans with some kind of data structure which holds only 2 values, which in C++ is the `std::pair`, one part of the pair would be a boolean and would decide whether there is a connection between the two nodes and the other value would be the weight. It is also possible to set some default value to the weight such as the max value of an integer with `std::numeric_limits<int>::max()`, therefore avoiding the need for a boolean.<br>
As I am using the vector of vertices, I created a function which converts such a structure to an adjacency matrix.
```c++
std::vector<std::vector<std::pair<bool, int> > > get_adjacency_matrix(std::vector<Vertex*> g) {
    std::vector<std::pair<bool, int> > temp(g.size(), std::make_pair(false, 0));
    std::vector<std::vector<std::pair<bool, int> > > r(g.size(), temp);
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            r[i][g[i]->adj(j)->val()] = std::make_pair(true, g[i]->weight(j));
        }
    }
    
    return r;
}
```
  While the adjacency matrix produced is much faster than any other structure which could be used as it offers constant look-up time, it lacks in the department of space complexity as it is $`O(N^2)`$.
  
### Adjacency List
An adjacency list consists of a list of vectors where each ith place in the list represents one vertex. In each slot there is a vector which holds all of the neighbours by using the `std::pair` and holding the index of the node and the weight required to reach it. To transform from a vector of vertices to an adjacency list you could use the following code:
```c++
#include "../vertex.hpp"

std::vector<std::vector<std::pair<int, int> > > get_adjacency_list(std::vector<Vertex*> g) {
    std::vector<std::vector<std::pair<int, int> > > r;
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            r[i].emplace_back(g[i]->adj(j)->val(), g[i]->weight(j));
        }
    }
    
    return r;
}
```
This method of storing a graph is quite efficient and actually pretty similar to the method used in this repository. Its lookup time is at worse $`O(N)`$, where N is the number of nodes, however, it is practically less and it is usually much more space-efficient than the adjacency matrix, with space complexity of $`O(E)`$, where E is the number of edges. Of course, E can be at worse equal to N^2, but usually that is not the case. So even though the adjacency matrix and list may at first glance take up the same amount of space, the fact that the list version uses vectors with adjustable length contributes to an overall decrease in size by increasing some of its time complexity.

  ### Edge List
Finally, another popular method of storing graphs is the edge list. Basically, with this method you store in a list all of the edges present in the graph without caring about the nodes. Each edge in this list is represented by three values, two of them are the values of the two vertices connected and the final one is the weight of the edge. To accomplish this I just created another custom struct. The code for the conversion between the vector of vertices and the edge list is:
```c++
struct edge {
    int a, b, w;
};

std::vector<edge> get_edge_list(std::vector<Vertex*> g) {
    std::vector<edge> r;
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            edge e;
            e.a = g[i]->val();
            e.b = g[i]->adj(j)->val();
            e.w = g[i]->weight(j);
            r.push_back(e);
        }
    }
    std::sort(r.begin(), r.end(), [](const auto &a, const auto &b) {
        return a.w < b.w;
    });
    return r;
}
```
With this way of storing, nodes with no edges will be completely ignored, which is quite the drawback in certain cases and it even has a space and time complexity of $`O(E)`$, where E is the number of edges. However, its main strength stems from its easy-to-manipulate order of the edges, making it extremely useful in some cases such as in [Kruskal's Algorithm](#kruskal).
  ## Searching

  ### Breadth-First Search

  ### Depth-First Search


  ## Shortest Path

  ### Dijkstra

  ### Bellman-Ford

  ### Floyd-Warshall
  

  ## Topological Sorting

  ### Using DFS

  ### Kahn's Algorithm
  

  ## Minimum Spanning Tree

  ### Kruskal

  ### Prim
  

  ## Strongly Connected Components

  ### Tarjan's Algorithm


  
