# Graph-Algorithms
Explanation and implementation of a bunch of different graph-related algorithms.

## Table of Contents
* [Introduction](#introduction)
    - [What are Graphs](#what-are-graphs)
    - [About this Repository](#about-this-repository)
* [Graph Representation](#graph-representation)
    - [Vector of Vertices](#vector-of-vertices)
    - [Adjacency Matrix](#adjacency-matrix)
    - [Adjacency List](#adjacency-list)
    - [Edge List](#edge-list)
* [Traversal](#traversal)
    - [Breadth-Frst Search](#breadth-first-search)
    - [Depth-First Search](#depth-first-search)
* [Pathfinding](#pathfinding)
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

## Introduction
### What are Graphs
A graph is a structure made up of <strong>vertices and edges</strong>. Vertices, also known as nodes, are connected through the edges. Edges can carry <strong>weights</strong> which are representeed by some number, representing some sort of consequence to get from node A to node B. Graphs can be used to solve a variety of different problems. They are even used in neural networks.

### About this Repository
In this repository I provide an explanation and the code for each graph algorithm. The <em>graph.cpp</em> showcases how they should be used. I used C++17 to compile it. Specifically you should run:
```bash
$ g++ graph.cpp -std=c++17
```
Also versions that come after C++17 should work. So you could compile with C++20 instead of C++17. It should also work just fine with C++14.<br>

I already had studied these algorithms beforehand so I just used [GeekForGeeks](https://www.geeksforgeeks.org) as a refresh while writing this repository. Also I found [this video](https://www.youtube.com/watch?v=wUgWX0nc4NY) very useful when it came to explaining [Tarjan's Algorithm](#tarjans-algorithm).

## Graph Representation
There are many ways to represent a graph in programming, and choosing the right one depends on your purpose. In the code of this repository, I will usually be using a vector composed of the nodes of the graph, where each node will be represented using a class. However, there are also other ways to accomplish the same thing, each with its own advantages and disadvantages.

### Vector of Vertices
I am going to be using a <strong>class</strong> to represent a single vertex throughout most of this repository and in some cases put each of the nodes in a vector. Basically, I created a class to act as a node which holds its <strong>value</strong>, a <strong>pointer</strong> to each neighbouring node as well as the <strong>weight</strong> of the edge connecting them and a way to add edges from one node to another. The code for the node is the following:
  
```c++
class Vertex {
    private:
        int value; // value of node
        std::vector<std::pair<Vertex*, int> > nextVertex; //pair of adjacent node and weight of edge connection
    public:
        //void function used to set the value of the node to an integer
        void val(int v) {
            value = v;
        }

        //function used to return the value of the node
        int val() {
            return value;
        }

        //void function used for adding a relationship between 2 nodes. The node is added as a neighbour with a weight of w(directed graph)
        void add(Vertex* n, int w) {
            nextVertex.emplace_back(n, w);
        }

        //gets the ith neighbour
        Vertex* adj(int i) {
            return nextVertex[i].first;
        }

        //gets the weight to get to the ith neighbour
        int weight(int i) {
            return nextVertex[i].second;
        }

        //returns total number of neighbours
        int len() {
            return nextVertex.size();
        }
};
```

### Adjacency Matrix
An adjacency matrix is basically a <strong>2D board</strong> of size VxV, where V is the number of vertices present in the graph. A number is given to each vertex and the place [i][j] in the board symbolises that there is an edge connecting node i and j. In cases where the weight of all edges is constant then a simple 2D boolean board would be enough, however, in cases with varying weights it is necessary to replace booleans with an integer symbolising the weight of the edge, and having some kind of default value to represent an absence of an edge. For instance, in C++ I just used the `limits` library and used `std::numeric_limits<int>::max()` as a default value which holds the maximum integer value of an int.<br>
As I am using the vector of vertices, I created a function which converts such a structure to an adjacency matrix.
```c++
std::vector<std::vector<int> > get_adjacency_matrix(std::vector<Vertex*> g) {
    //temp represents a single row
    std::vector<int> temp(g.size(), std::numeric_limits<int>::max());
    //r is the final result
    std::vector<std::vector<int> > r(g.size(), temp);
    for (int i = 0; i<g.size(); i++) {
        //distance from node i to itself is 0
        r[i][i] = 0;
        //loop through all neighbours of i
        for (int j = 0; j<g[i]->len(); j++) {
            //add an edge in the adjacency matrix
            r[i][g[i]->adj(j)->val()] = g[i]->weight(j);
        }
    }
    
    return r;
}
```
While the adjacency matrix produced is much faster than any other structure which could be used as it offers constant look-up time, it lacks in the department of space complexity as it is $O(V^2)$, with V being the number of vertices.
  
### Adjacency List
An adjacency list consists of a <strong>list of vectors</strong> where each <em>ith</em> place in the list represents one vertex. In each slot, there is a vector which holds all of the neighbours by using the `std::pair` and holding the index of the node and the weight required to reach it. To transform from a vector of vertices to an adjacency list you could use the following code:
```c++
std::vector<std::vector<std::pair<int, int> > > get_adjacency_list(std::vector<Vertex*> g) {
    //r represents the final result
    std::vector<std::vector<std::pair<int, int> > > r;
    //loop through all nodes and their neighbours and add the approriate connections
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            r[i].emplace_back(g[i]->adj(j)->val(), g[i]->weight(j));
        }
    }
    
    return r;
}
```
This method of storing a graph is quite efficient and actually pretty similar to just using a vector of vertex objects. Its lookup time is at worse $O(V)$, where V is the number of nodes, however, it is practically less and it is usually much more space-efficient than the adjacency matrix, with space complexity of $O(E)$, where E is the number of edges. Of course, E can be at worse equal to $V^2$, but usually, that is not the case. So even though the adjacency matrix and list may at first glance take up the same amount of space, the fact that the list version uses vectors with adjustable length contributes to an overall decrease in size by increasing some of its time complexity.

### Edge List
Finally, another popular method of storing graphs is the edge list. Basically, with this method you store in a <strong>list all of the edges</strong> present in the graph without caring about the nodes. Each edge in this list is represented by three values, two of them are the values of the two vertices connected and the final one is the weight of the edge. To accomplish this I just created another <strong>custom struct</strong>. The code for the conversion between the vector of vertices and the edge list is:
```c++
//a is the first node, b is the second node and w is the weight
struct edge {
    int a, b, w;
};

std::vector<edge> get_edge_list(std::vector<Vertex*> g) {
    //final result is saved in r
    std::vector<edge> r;
    //loop through all of the nodes and their neighbours and add an edge to the edgelist
    for (int i = 0; i<g.size(); i++) {
        for (int j = 0; j<g[i]->len(); j++) {
            edge e;
            e.a = g[i]->val();
            e.b = g[i]->adj(j)->val();
            e.w = g[i]->weight(j);
            r.push_back(e);
        }
    }
    return r;
}
```
With this way of storing, nodes with no edges will be completely ignored, which is quite the drawback in certain cases and it even has a space and time complexity of $O(E)$, where E is the number of edges. However, its main strength stems from its easy-to-manipulate order of the edges, making it extremely useful in some cases such as in [Kruskal's Algorithm](#kruskal).

## Traversal
Graph traversal refers to the <strong>process of visiting all nodes in a graph</strong>. This process can be carried out for a multitude of reasons, such as searching for a specific node. Two of the most popular techniques in accomplishing this are Breadth-First Search and Depth-First Search. Traversal algorithms are crucial to graph theory as they are used time and time again to accomplish other much more complex tasks.

### Breadth-First Search
Breadth-First Search, also known as BFS, is a traversal algorithm. It works by starting its search at a single node, marking it as visited, and then, using a queue adds all unvisited neighbours of the node to the queue. Then, the next vertex is taken from the <strong>queue</strong> and the process repeats until the queue becomes empty. For instance, if we had the following graph:
```mermaid
graph TD;
A["1"] --> B["2"]
A --> C["3"]
B --> D["4"]
B --> C
```
If we call BFS on this graph, starting at node 1, then the algorithm will mark node 1 as visited and add nodes 2 and 3 to its queue.
```mermaid
graph TD;
subgraph visited
A["1"]
end
subgraph queue
B["2"]
C["3"]
end
A --> B
A --> C
B --> D["4"]
B --> C
```
Then it will access the queue. Let's say that node 2 was added first. That means that node 2 is marked as visited and then node 4 is added to the queue. However, node 3 has already been added to the queue and is therefore not added again. So the graph will now look like this:
```mermaid
graph TD;
subgraph visited
A["1"]
B["2"]
end
subgraph queue
C["3"]
D["4"]
end
A --> B
A --> C
B --> D["4"]
B --> C
```
Then, node 3 is accessed and marked as visited, however, it has no neighbours to add to the queue. Then, the loop runs again one final time for node 4 which is also marked as visited and with no neighbours to add and no other items on the queue, the BFS stops.<br>

With that algorithm in mind, we can now construct our program:
```c++
Vertex* bfs(Vertex *head, int key) {
    std::queue<Vertex*> q;
    q.push(head);
    std::unordered_map<Vertex*, bool> visited;
    //while the queue is not empty
    while (!q.empty()) {
        Vertex* current = q.front();
        //return the current node if value is found
        if (current->val() == key) return current;
        q.pop();
        //if current node has been visited then ignore it
        if (visited[current]) continue;
        //mark as visited
        visited[current] = true;
        int l = current->len();
        for (int i = 0; i < l; i++) {
            Vertex* temp = current->adj(i);
            //if we haven't visited the neighbour, push them to the queue
            if (visited.find(temp) == visited.end()) {
                q.push(temp);
            }
        }
    }
    //return nullptr as there is no node of value key
    return nullptr;
}

```
The above algorithm uses BFS to search for a specific node with a value of `key`. The `head` is just the starting node passed on to it. It returns a pointer to the node if it is found, otherwise, it returns `nullptr`. The time complexity of the algorithm is $O(V+E)$, where V is the number of vertices and E is the number of edges. This is because each vertex is queued and dequeued only once and each edge is checked only once, giving us the above time complexity. When it comes to space complexity, the queue can hold at most V items which means that the space complexity is $O(V)$.

### Depth-First Search
Depth-First Search, also known as DFS, is a graph traversal algorithm very similar to BFS. However, instead of utilising a queue for storing the neighbours of the current vertex, it utilises a <strong>stack.</strong> So if we take the previous example, on the first loop the graph will be the same and on the second loop it will also go down node 2 so the graph will be similar:
```mermaid
graph TD;
subgraph visited
A["1"]
B["2"]
end
subgraph stack
C["3"]
D["4"]
end
A --> B
A --> C
B --> D["4"]
B --> C
```
However, now that a stack is being used, the next item which will be picked will not be node 3, but node 4. Therefore the graph will become:
```mermaid
graph TD;
subgraph visited
A["1"]
B["2"]
D["4"]
end
subgraph stack
C["3"]
end
A --> B
A --> C
B --> D["4"]
B --> C
```
Finally, node 3 will be checked and the program will be terminated.<br>
Because the only change made is for the queue to be swapped with a stack, we can simply take the previous BFS code and change it up a bit like so:
```c++
//same exact code as BFS but substituted the queue with a stack
Vertex* dfs(Vertex *head, int key) {
    std::stack<Vertex*> q;
    q.push(head);
    std::unordered_map<Vertex*, bool> visited;
    while (!q.empty()) {
        Vertex* current = q.top();
        if (current->val() == key) return current;
        q.pop();
        if (visited[current]) continue;
        visited[current] = true;
        int l = current->len();
        for (int i = 0; i < l; i++) {
            Vertex* temp = current->adj(i);
            if (visited.find(temp) == visited.end()) {
                q.push(temp);
            }
        }
    }
    return nullptr;
}
```
Once again, the `key` represents the value that we are looking for and the `head` is the starting node. If the `key` is not found then `nullptr` will be returned. The time and space complexity of this DFS are the same as BFS, so the asymptotic time notation is $O(V+E)$ and the space complexity is $O(V)$, where V is the number of vertices and E is the number of edges. 

#### Recursive DFS
The two programs look almost identical and have the same time and space complexities, so what is the point in using one over the other? It's better to use BFS when the answer you are looking for is closer to the source node, whereas it is more optimal to use DFS when the answer lies somewhere quite far away from the source node. However, even then, most  people would rather go for DFS as its recursive solution is much easier to implement:
```c++
Vertex* dfs_recursive(Vertex *head, int key, std::unordered_map<Vertex*, bool>& visited) {
    //2 base cases, when we found the key we return the current node and when we reach an already visited node we return nullptr
    if (head->val() == key) return head;
    if (visited[head]) return nullptr;
    //mark as visited
    visited[head] = true;
    int l = head->len();
    for (int i = 0; i < l; i++) {
        Vertex* temp = head->adj(i);
        //if the neighbour is not visited then call DFS recursively on it
        if (visited.find(temp) == visited.end()) {
            Vertex* r = dfs_recursive(temp, key, visited);
            //if there was a result then return the value
            if (r != nullptr) return r;
        }
    }
    //last base case when nothing is found we return nullptr
    return nullptr;
}
```
Again, the way it works and the input it receives is the same as the iterative DFS, however, it has an extra parameter which is the visited map passed on as a reference. Its space and time complexity are the same, however, its space complexity does not come from the stack structure, but rather from the recursive calls. Of course, we can overload the function to exclude the third parameter from the first function call like so:
```c++
//overloaded to use less parameters when called
Vertex* dfs_recursive(Vertex *head, int key) {
    std::unordered_map<Vertex*, bool> visited;
    return dfs_recursive(head, key, visited);
}
```

## Pathfinding
Pathfinding algorithms are used to figure out, as the name suggests, the optimal path between two nodes on a graph. In cases where the weight of all of the edges is the same, a simple BFS would be enough to calculate the shortest path, however, with the addition of weights, such an approach is insufficient.<br>

These pathfinding algorithms can be used in real life to calculate the fastest or shortest route between 2 points on a map. However, that is just the most obvious example. They can also be used to calculate optimal choices where each edge leads to a new one and the weight of an edge is some kind of consequence. For instance, each node could represent a state and you want to get from state X to state Y. This could easily translate to finding solutions to a Rubik's cube or even at a Tic-Tac-Toe game where you would want to solve the Rubik's cube or go into a win state respectively. 

### Dijkstra
Dijkstra is a pathfinding algorithm which only works on graphs with <strong>non-negative weights</strong>. It closely resembles BFS, with the greatest change being the use of a <strong>priority queue</strong> instead of a queue. In C++, a priority queue(`std::priority_queue`) is implemented using <strong>heaps</strong>. A heap is a binary tree in which every parent node has the minimum/maximum value between their children. There are 2 types of heaps, max-heaps and min-heaps. Max-heaps have parents with maximum value whereas min-heaps have them with minimum value. In the case of Dijkstra, we will be using a min-heap. With heaps, you can access the minimum element of all of the items inserted in constant time. However, the addition of elements to the heap has an asymptotic time notation of $log(N)$, where N is the amount of items added.<br>
Dijkstra does not only find the shortest path from node A to B, but in reality, it finds the shortest path from a source node to all other nodes in a graph. The way it works is simple. The steps are described below:
1. Using a hash map or vector mark the source node as having a distance of 0 from the source and all other nodes as having infinite distance(to imply they are unvisited)
2. Push the source node to the priority queue together with the distance from the source node(0)
3. Use the priority queue to select the node closest to the source node
4. Mark the distance from the source node to this node as it has now been visited and the minimum distance has been found
5. Add all neighbours of the node to the priority queue together with their distance from the source Node
6. Repeat steps 3 - 5 until the priority queue is empty or the target node has been reached<br>

Basically, what happens in the above steps is that we add all neighbours of visited nodes to a priority queue together with the required distance to get to them from the source node by cumulating weights from the path required to get there. Then we pick the one with the least total distance and mark it as visited as we know there is no possible way to get back to this node with a lower total path. We then add those neighbours to the priority queue and continue.<br>
For example, let's say we have the following graph:
```mermaid
graph TD;
A["A, 0"] -->|2| B["B, ∞"]
A -->|6| C["C, ∞"]
B -->|4| D["D, ∞"]
B -->|2| C
C -->|5| B
```
Let's say we are looking for the minimum distance between node A and D. If the source node is A, then we mark it as visited with a distance of 0 and we then push onto the priority queue its neighbours together with the weight required to get to them. The priority queue chooses the path to node B as it has a weight of 2. It is marked as visited with a distance from A of 2.
```mermaid
graph TD;
subgraph visited
A
B
end
A["A, 0"] ==>|2| B["B, 2"]
A -->|6| C["C, ∞"]
B -->|4| D["D, ∞"]
B -->|2| C
C -->|5| B
```
Then, we check B's neighbours. We push them onto the priority queue. We then access from the priority queue node C, which has a total distance from A being 4. 2 from A to B and 2 from B to C.
```mermaid
graph TD;
subgraph visited
A
B
C
end
A["A, 0"] ==>|2| B["B, 2"]
A -->|6| C["C, 4"]
B -->|4| D["D, ∞"]
B ==>|2| C
C -->|5| B
```
Then following the same logic we access the next node on the priority queue which is node D.
```mermaid
graph TD;
subgraph visited
A
B
C
D
end
A["A, 0"] ==>|2| B["B, 2"]
A -->|6| C["C, 4"]
B ==>|4| D["D, 6"]
B ==>|2| C
C -->|5| B
```
And like that, we found our target node, D, which requires a total distance of 6 to get to from node A.<br>

With that logic in mind, we can construct the following code:
```c++
int dijkstra(Vertex *head, int key) {
    //priority queue using min-heap
    std::priority_queue<std::pair<int, Vertex*>, std::vector<std::pair<int, Vertex*> >, std::greater<std::pair<int, Vertex*> > > q;
    q.push(std::make_pair(0, head));
    std::unordered_map<Vertex*, int> weights;
    while (!q.empty()) {
        Vertex* current = q.top().second;
        int weight = q.top().first;
        if (current->val() == key) return weight;
        q.pop();
        //if new weight is worse than the previous one then break
        if (weight > weights[current]) break;
        weights[current] = weight;
        int l = current->len();
        //go through all of the node's neighbours and if the new weight is better then push them to the queue
        for (int i = 0; i < l; i++) {
            Vertex* temp = current->adj(i);
            int w = current->weight(i);
            if (weights.find(temp) == weights.end() || weight+w < weights[temp]) {
                weights[temp] = weight+w;
                q.push(std::make_pair(weight+w, temp));
            }
        }
    }
    return -1;
}
```
The function above returns the total distance required to get from the source node(`head`) to the target with a value of `key`. If it fails to find an answer it simply returns -1. The time complexity of Dijkstra is $O((V+E)log(V))$, where V is the number of vertices and E is the number of edges. The reason behind this time complexity comes from the fact that each vertex will be extracted once from the priority queue, and we will have at most E amount of insertions in the priority queue. And since both of those operations take $O(log(V))$ time, then once multiplied by the number of times they are carried out, you get the above time complexity. That time complexity can be improved down to $O(V + E \times log(V))$ when updating the priority queue instead of adding an edge each time. And it can even be improved further by implementing a Fibonacci heap for the priority queue instead of a binary heap, leading to a $O(E + V \times log(V))$. On the other hand, the space complexity is on average $Θ(V)$, and rarely in the worst case it can be $O(V^2)$.

### Bellman-Ford
Bellman-Ford is a pathfinding algorithm which finds the shortest paths from a source node to all other nodes and it works on graphs with <strong>negative and non-negative weights</strong>. Bellman-Ford <strong>fails to work for graphs with negative cycles</strong>, however, it can detect whether there are any, which is pretty important.<br>

The idea behind the algorithm is quite simple. First, it marks every node as having an <strong>infinite distance</strong> from the source node. Then it goes through all of the nodes and checks if the distance from the source node plus the weight of an edge with a neighbour is less than the distance that this neighbour has with the source node. It does this V-1 times or until there is no change in the distance of any node. It is guaranteed that after V-1 times the shortest distance will be found between the starting node and all other nodes because the shortest path between any two nodes can have at most V-1 edges, at it can at most pass through every node once. However, in cases where a negative weight cycle is introduced, this reasoning becomes false as it can pass through that cycle an infinite number of times. Therefore, if changes in the distances of nodes keep on appearing even after n-1 repetitions, then that means that a negative cycle exists.<br>

For instance, if we have the following graph and the source node is A:
```mermaid
graph TD;
A["A, 0"] -->|2| B["B, ∞"]
A -->|6| C["C, ∞"]
B -->|4| D["D, ∞"]
B -->|2| C
C -->|5| B
```
Bellman-Ford will loop through all nodes and change the values of their neighbours, in this case:
```mermaid
graph TD;
A["A, 0"] -->|2| B["B, 2"]
A -->|6| C["C, 6"]
B -->|4| D["D, ∞"]
B -->|2| C
C -->|5| B
```
Then it will do it again:
```mermaid
graph TD;
A["A, 0"] -->|2| B["B, 2"]
A -->|6| C["C, 4"]
B -->|4| D["D, 6"]
B -->|2| C
C -->|5| B
```
And it will check again, but this time, there are no changes in the graph which goes to show that the optimal distances have already been found and the algorithm terminates.

With that algorithm in mind, we can create the following code:
```c++
std::vector<int> bellman_ford(std::vector<Vertex*>& arr, int start) {
    //default weight set to the maximum integer an int can hold
    const int defaultWeight = std::numeric_limits<int>::max();
    std::vector<int> total(arr.size(), defaultWeight);
    total[start] = 0;
    int counter = 0;
    bool changed = true;
    //continues loop while there is a change in values and it has not gone over V-1 repetitions
    while (changed && counter < arr.size()-1) {
        changed = false;
        for (int i = 0; i<arr.size(); i++) {
            //if there is no path currently then continue
            if (total[i] == defaultWeight) continue;
            int l = arr[i]->len();
            //go through all of the node's neighbours and recalculate optimal routes
            for (int j = 0; j<l; j++) {
                int v = arr[i]->adj(j)->val();
                if (total[i] + arr[i]->weight(j) < total[v]) {
                    changed = true;
                    total[v] = total[i] + arr[i]->weight(j);
                }
            }
        }
        counter++;
    }
    return total;
}
```
In this case, I ended up using the vector of vertices as it allows me to easily access all of the nodes and their respective neighbours so that I can do the <strong>relaxation</strong> of edges. The algorithm has a time complexity of $O(E*V)$, where V is the number of vertices and E is the number of edges. Of course, this is the case because in the worst-case scenario in which every node is connected to every other node, we would have at most $V-1$ loops where in each loop we would have $V-1$ edges to go through, which makes a total of $(V-1)^2$ operations, which is basically equal to $O(V^2)$ in the Big-O notation.<br>

#### Negative Cycle Detection
The time complexity of Bellman-Ford is obviously worse than that of Dijkstra, however, its main strength lies in its ability to detect negative cycles. Using the Bellman-Ford algorithm, you could detect negative cycles by slightly modifying the previous code:
```c++
//same as before but instead of stopping the while loop at V-1 it stops at V and returns the changed varaible
bool has_negative_cycle(std::vector<Vertex*>& arr, int start) {
    const int defaultWeight = std::numeric_limits<int>::max();
    std::vector<int> total(arr.size(), defaultWeight);
    total[start] = 0;
    int counter = 0;
    bool changed = true;
    while (changed && counter < arr.size()) {
        changed = false;
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
        counter++;
    }
    
    return changed;
}
```

### Floyd-Warshall
Floyd-Warshall algorithm finds the shortest path between <strong>all pairs of nodes</strong> in a graph and works on both <strong>directed and undirected graphs</strong>. However, it <strong>fails to work when a negative cycle</strong> is included within the given graph.<br>

It takes advantage of a concept known as <strong>dynamic programming</strong>. It works by using an <strogn>adjacency matrix</strong> and storing the distances from node i to node j in each slot. The values are initialised by inputting the edges into the matrix, and if no edge exists between two nodes then the value is set to infinity. That is the initialisation process. The actual algorithm takes into account, that for the shortest path between node i to j, there can be k intermediate nodes between the path, and it therefore loops over all possible source, target and intermediate nodes and updates the shortest path in the matrix `dists[i][j]`, only if `dists[i][j] > dists[i][k] + dists[k][j]`. After all three nested loops end, the matrix will contain all shortest paths between all pairs of nodes.<br>

The code for such a simple algorithm would just look like this:
```c++
std::vector<std::vector<int> > floyd_warshall(std::vector<std::vector<int> >& dists) {
    const int defaultMax = std::numeric_limits<int>::max();
    //first loop is of the intermediate node, then the source node and then the target node
    for (int k = 0; k<dists.size(); k++) {
        for (int i = 0; i<dists.size(); i++) {
            for (int j = 0; j<dists.size(); j++) {
                //if the new distance is better then update the optimal one
                if (dists[i][k] != defaultMax && dists[k][j] != defaultMax && dists[i][j] > dists[i][k] + dists[k][j]) {
                    dists[i][j] = dists[i][k] + dists[k][j];
                }
            }
        }
    }
    return dists;
}
```
The input is just the adjacency matrix, and the rest of the code is composed of 3 nested loops, giving us a time complexity of $O(V^3)$, where V is the number of vertices present in the graph. The space complexity is just $O(V^2)$ because we are using an adjacency matrix with dimensions $V \times V$.

## Topological Sorting
Topological sorting is the ordering of the nodes in a graph, such that for <strong>every pair of vertices (u, v), node u comes before v in the list if an edge comes from u to v</strong>. For this to be possible, topological ordering can only be applied to <strong>directed acyclic graphs(DAG)</strong>. Of course, the definition of topological sorting implies that there can be <strong>more than just one</strong> topological sorting for a given graph, however, returning just one is enough.<br>

Topological sorting is usually used as a way to order events, ensuring that something happens before another thing happens if necessary. For instance, it could be used for task scheduling where you need to perform a task before another one and so topological sorting would be able to help you figure out the right order of accomplishing tasks. Other more complicated usages include dependency resolution where dependencies are resolved in the right order. It can even be used in compiler optimizations and many more.

### Using DFS
One way to accomplish topological sorting is by using DFS. We can create a <strong>hash map</strong> to keep track of all of the nodes that have been visited. At the start, we <strong>mark everything as unvisited</strong> and we loop through all of them. Each time we find one that is unvisited, we call a DFS which basically goes through all of the unvisited neighbours of the called node and it calls dfs on them. This happens <strong>recursively</strong> and once a node is done with all of its neighbours it is then <strong>pushed</strong> into the results list. After all of the vertices have been visited we just <strong>reverse the ordering</strong> of the list and the resulting list is our topologically sorted graph. Reversing the list is required because we push the visited nodes after all their neighbours have been explored, which means that the first nodes in the list are those with 0 neighbours.<br>
The code would look like this:
```c++
//helper function
void dfs_topo(Vertex* v, std::stack<Vertex*>& s, std::unordered_map<Vertex*, bool>& visited) {
    //mark current node as visited
    visited[v] = true;
    int len = v->len();
    //call DFS recursively for all unvisited neighbours
    for (int i = 0; i<len; i++) {
        if (visited.find(v->adj(i)) == visited.end()) {
            dfs_topo(v->adj(i), s, visited);
        }
    }
    //add node to the result
    s.push(v);
}

//main function
std::vector<Vertex*> dfs_topological(std::vector<Vertex*>& g) {
    std::stack<Vertex*> s;
    std::unordered_map<Vertex*, bool> visited;
    //perform DFS on all nodes which have not been visited before
    for (int i = 0; i<g.size(); i++) {
        if (visited.find(g[i]) == visited.end()) {
            dfs_topo(g[i], s, visited);
        }
    }
    //reverse the result and print it
    std::vector<Vertex*> r;
    for (int i = 0; i<g.size(); i++) {
        r.push_back(s.top());
        s.pop();
    }
    return r;
}
```
The main function is the `dfs_topological` and the helper function is `dfs_topo`. The main function for topological sorting is called with only just one parameter, a vector of vertices. The time complexity is $O(V+E)$, where V is the number of vertices and E is the number of edges, due to the usage of DFS. The space complexity is $O(V)$, as you need to at least have a vector of size V containing the topologically sorted graph.

### Kahn's Algorithm
Kahn's algorithm is very simple and uses a <strong>queue</strong> for its solution. First off, it notes the <strong>in-degree</strong>(amount of edges pointing to the node) of all of the vertices. Then it pushes to the queue all of the vertices with an <strong>in-degree of 0</strong>. Then it removes from the queue the first element, pushes it to the topologically ordered list and substracts 1 from all of the in-degrees of the neighbours of that vertex. If any of them reach an in-degree of 0 they are also added to the stack. This goes on until the queue becomes empty. If the result is less than the number of nodes provided then that means that the graph given was not a DAG. Otherwise, if the size of the 2 structures matches, it means that the algorithm was successful and it returns the result.<br>
The code would therefore be:
```c++
std::vector<Vertex*> kahn(std::vector<Vertex*>& g) {
    std::unordered_map<Vertex*, int> in;
    //set the in-defree of each node by going through each node's neighbours
    for (int i = 0; i<g.size(); i++) {
        int l = g[i]->len();
        for (int j = 0; j<l; j++) {
            if (in.find(g[i]->adj(j)) == in.end()) in[g[i]->adj(j)] = 1;
            else in[g[i]->adj(j)]++;
        }
    }

    //push to the queue nodes with in-degree of 0
    std::queue<Vertex*> q;
    for (int i = 0; i<g.size(); i++) {
        if (in.find(g[i]) == in.end()) q.push(g[i]);
    }

    std::vector<Vertex*> r;
    //while queue is not empty decrease the in-degree of front node's neighbours and add it to the result
    while (!q.empty()) {
        Vertex* v = q.front();
        q.pop();
        r.push_back(v);
        int l = v->len();
        for (int i = 0; i<l; i++) {
            if ((--in[v->adj(i)]) == 0) q.push(v->adj(i));
        }
    }
    //if the result does not include all nodes then the graph was not a DAG
    if (r.size() != g.size()) {
        return {};
    }
    return r;
}
```
The function only contains one parameter, `g`, which is the graph represented as a vector of vertices. The code is of time complexity $O(V+E)$, where V is the number of vertices and E is the number of edges. This is because initialising all the in-degrees of all of the nodes happens in $O(E)$ time, then picking all of the nodes with an in-degree of zero requires $O(V)$ time, then subtracting the in-degree of all of the vertices takes again $O(E)$ time and popping all of the items from the queue takes another $O(V)$ time. Checking for if the graph is not a DAG at the end takes constant time, so in total, the time complexity ends up being $O(V+E)$. The space complexity is just $O(V)$ due to the queue, the final result and the map keeping track of the in-degree, all three of which take up $O(V)$ size.

## Minimum Spanning Tree
A Minimum Spanning Tree(MST) is a <strong>subgraph of a weighted undirected graph which connects all vertices without creating any cycles and with the minimum possible total edge weight</strong>. A graph can have <strong>multiple possible MSTs</strong> when not all edge weights are distinct. However, in cases where edge weights are all different then there can only be one unique MST. All MSTs, have a V number of vertices and a V-1 number of edges.<br>

MST algorithms are used to figure out the lowest-cost connections in water networks and even electrical grids while still keeping everything connected. This can save companies and governements loads of money while still having everything work as planned.

### Kruskal
Kruskal is a <strong>greedy algorithm</strong> which finds a minimum spanning tree by picking the <strong>lightest edges</strong> each time and adding them to the final result making sure that <strong>no cycle</strong> is created. It continues following these steps until all vertices have been connected. To figure out which edge is the lightest at each step, an edge list is used sorted based on weight in ascending order. However, the biggest problem arises from how to recognise whether or not a cycle will form if an edge is added. One way to check for that is if the added edge connects 2 nodes which are already connected. If it connects them, then a cycle will form, therefore the edge is ignored. To check whether or not 2 nodes are part of the same tree we need to use a <strong>Disjoint Set Union</strong>.<br>

#### Disjoint Set Union
A Disjoint Set Union(DSU) works by keeping track of a <strong>representative</strong> for each separate tree formed. At the start, all nodes are added with themselves being the representatives of their own one-node tree. Then, the DSU contains two methods, <strong>union-find</strong>. The method `find`, just <strong>returns the representative of the given node</strong>. It does so <strong>recursively</strong>, as the representative of its parent node might be a different node. It stops the recursion when the representative of the node is the node itself. However, to keep the `find` method fast, each time the method is called, it does not just return the final representative of the tree but also <strong>changes all the representatives</strong> along the way to the final representative therefore shortening the path to the tree's representative. This type of find takes advantage of a concept called <strong>Path Compression</strong>. Therefore, its code would look like this:
```c++
int find(int i) {
    //if this is not the representative then recursively find new representative and set the current one equal to the final one
    if (parents[i] != i) parents[i] = find(parents[i]);
    return parents[i];
}
```
The next method is `union`. It has 2 nodes as parameters and its objective is to <strong>merge the two different trees</strong> in which both nodes are part. To do that it calls the `find` method of both nodes and if they are not part of the same tree then the representative's representative of the first tree is set to be equal to the representative of the second tree. So the code for `union` would be:
```c++
void unite(int i, int j) {
    //find both representatives
    int first = find(i);
    int second = find(j);
    //if they are together then do nothing
    if (i == j) return;
    //put them together choosing node j's representative as the final representative
    parents[first] = second;
}
```
However, just like there was a trick to make `find` faster, there is also a trick to optimize further the `union` method by introducing <strong>ranks</strong> to each tree. The main problem of `union`, is that it does not necessarily set the optimal representative of the 2 trees as the best representative. It is generally better to change the <strong>smallest tree's representative to the bigger tree's representative</strong>. So the rank is similar to the <strong>depth</strong> of each tree, the only difference being that the rank is not affected by the optimisations in height done in the `find` method. The rank of each tree starts at 0. Each time we merge 2 trees with the same rank, we increase the rank of the final tree formed. This type of union is called <strong>union by rank</strong> and it is much faster than the original one. The code would be:
```c++
void unite(int i, int j) {
    int first = find(i);
    int second = find(j);
    if (i == j) return;
    //if the first tree has a greater rank then choose its representative for the merge
    if (ranks[first] > ranks[second]) {
        parents[second] = first;
    }
    else {
        //the second tree's representative is the new one
        parents[first] = second;
        //if they are the same rank then increase the rank of the second one
        if (ranks[first] == ranks[second]) {
            ranks[second]++;
        }
    }
}
```
In total, combining the 2 algorithms together along with their optimisations gives us an <strong>amortized</strong> time complexity of $O(α(n))$, where α(n) is the inverse Ackerman function which for all practical values of n does not grow past 4. This means that the amortized time complexity is basically constant. In the worst-case scenario, the time complexity can become $O(log(n))$, but due to the optimizations made in both functions, this time complexity is rarely realised as each call to the functions optimizes the "environment" for the next call.<br>

The final  code for the DSU class together with the initialisation is:
```c++
class DSU {
    private:
        std::vector<int> parents;
        std::vector<int> ranks;
    public:
        //constructor setting for each node the rank to 0 and the representative to itself
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
```
#### Kruskal Code
Having created the DSU class and having an idea of how Kruskal's algorithm works, we can now write the code for it:
```c++
int kruskal(std::vector<edge>& edges) {
    //if there are no edges then there is no MST
    if (edges.size() == 0) return 0;

    //sort the edges based on weight in ascending order
    std::sort(edges.begin(), edges.end(), [](const auto &a, const auto &b) {
        return a.w < b.w;
    });

    //creating required parameters for DSU
    std::vector<int> f;
    for (int i = 0; i<edges.size(); i++) {
        f.push_back(i);
    }
    DSU dsu(f);
    
    int total = 0;
    //pick the next smallest edge each time without creating any loops by using the DSU to check for them
    for (edge e: edges) {
        if (dsu.find(e.a) != dsu.find(e.b)) {
            dsu.unite(e.a, e.b);
            total += e.w;
        }
    }
    return total;
}
```
The above code for Kruskal has one parameter, the edge list, and it returns the total edge weight of the minimum spanning tree. The time complexity is $O(E \times log(E))$ from the sorting, $O(E)$ from creating the DSU and $O(E \times α(V))$ from the final creation of the MST. In total, if you add them all up you'll get a time complexity of $O(E \times log(E) + E + E \times α(V)) = O(E \times log(E))$, where E is the number of edges and V is the number of vertices in the graph. The space complexity is $O(E + V)$ due to the edge list taking up $O(E)$ space and the DSU taking up $O(V)$ space, where E is the number of edges and V is the number of vertices.

### Prim
Prim's algorithm is a <strong>greedy algorithm</strong>. It starts from a single <strong>source node</strong> and it repeatedly chooses the <strong>next smallest edge</strong> from the available edges making sure no cycle is created by eliminating any edges which connect to already visited nodes. This goes on until all vertices have been visited. To check whether a vertex has already been visited you can just use a hash map or a simple vector. For always choosing the minimum weighted edge Prim's algorithm takes advantage of a <strong>priority queue</strong> just like Dijkstra. Therefore, the code would look like this:
```c++
int prim(std::vector<Vertex*> g) {
    int total = 0;
    int visitedAmount = 0;
    std::vector<bool> visited(g.size(), false);
    //priority queue using min-heap
    std::priority_queue<std::pair<int, Vertex*>, std::vector<std::pair<int, Vertex*> >, std::greater<std::pair<int, Vertex*> > > pq;
    pq.emplace(0, g[0]);
    //pick each time the next lightest available edge, go through all neighbouring edges and add them to the priority queue when leading to unvisited nodes
    while (visitedAmount < g.size()) {
        Vertex* current = pq.top().second;
        visited[current->val()] = true;
        total += pq.top().first;
        visitedAmount++;
        pq.pop();
        int l = current->len();
        for (int i = 0; i<l; i++) {
            if (!visited[current->adj(i)->val()]) pq.emplace(current->weight(i), current->adj(i));
        }
    }
    return total;
}
```
The code above takes as input a vector of vertices representing the graph and returns the size of the minimum spanning tree. The algorithm is very similar to that of [Dijkstra's](#dijkstra) and therefore their complexities are quite similar. If we symbolise the number of vertices with V and the number of edges with E then the space complexity is $O(V+E)$ because it needs $O(V)$ space from keeping track of visited nodes and another $O(E)$ space for the priority queue. Once again, the time complexity is $O((V+E) \times log(V))$, however, it can be improved all the way down to $O(E + V \times log(V))$ using Fibonacci heap and decrease-key operations just like Dijkstra.

## Strongly Connected Components
Strongly Connected Components(SCCs) are <strong>maximal subgraphs in which all vertices can access all other vertices</strong>. SCCs only apply to <strong>directed graphs</strong>. SCCs are similar to cycles but not the same. A simple DFS would not suffice for this task as it can inform us of which nodes are accessible from a single node but it cannot tell us whether the opposite is also true.<br>

SCCs can be used to figure out close friends on social media by looking at all of their contacts or even looking at a group of people and figuring out a common link between all of them which could help in advertising to a specific target audience. Another practical use is to ensure that a road network is strongly connected because if it was not then it would be possible for someone to get stuck in a particular place in the graph as there would be a way in but no way out.

### Tarjan's Algorithm
Tarjan's algorithm is capable of figuring out all of the SCCs within a graph. The way it does so is by utilising an <strong>altered version of a DFS</strong>. The difference is that it keeps a <strong>unique ID</strong> for each node visited starting from an ID of 0 and going up by one each time a new node is explored. Also, it keeps track of a <strong>low-link value</strong> for each node which is basically the <strong>lowest ID of a node accessible</strong> from the current node. This low-link value is initially set to be equal to the ID of the node, but, every time the DFS is done exploring a single neighbour of the node it updates its low-link value to the <strong>minimum low-link value between its own and its neighbour's</strong>, only if the neighbour was <strong>previously unexplored</strong> and <strong>not part of another SCC</strong>. This continues until the low-link value of a node matches its own ID and there are no more neighbours to explore. This means that the current node is part of an SCC and therefore it should note this node and all the nodes accessible from its neighbours which had the same low-link value as part of the same SCC. To keep track of these other neighbours a <strong>stack</strong> can be used so that every time a DFS is called on a node it is added to the stack. Then on the backtracking of the DFS when we reach a node with equal ID and low-link value we can simply start removing those items from the stack until we reach the current node. The items removed from the stack are all part of the same SCC. Furthermore, this stack is useful as it can be used to determine if another node has already been added to another SCC, because if it is visited and not part of the stack then it goes to show that the specific node is actually already part of an SCC.<br>

This algorithm can be summarised as follows:
* Mark all nodes as unvisited
* For each unvisited node perform DFS:
    - Push node to the stack
    - Set its ID to the next lowest possible value
    - Set its low-link value equal to its ID
    - Loop through each neighbour:
        * If the neighbour is unvisited call DFS on them
        * If the neighbour is on the stack then set the current low-link value equal to the minimum low-link value of the neighbour and current node
    - If the ID is equal to the low-link value then:
        * Start popping nodes on the stack until you reach the current one
        * Increase the number of SCCs by one
* Return the number of SCCs found<br>

With those steps in mind, we can now construct the required code:
```c++
void dfs_t(Vertex* head, std::unordered_map<Vertex*, int>& lows, std::unordered_map<Vertex*, int>& ids, int& amount, std::unordered_map<Vertex*, bool>& onStack, std::stack<Vertex*>& s, int& id) {
    //add node to the stack of being explored
    s.push(head);
    onStack[head] = true;
    //give it an id and set its low-link value to its id
    lows[head] = id;
    ids[head] = id++;
    
    int len = head->len();
    for (int i = 0; i<len; i++) {
        Vertex* temp = head->adj(i);
        //if the neighbour is univisted, recursively call DFS
        if (ids.find(temp) == ids.end()) dfs_t(temp, lows, ids, amount, onStack, s, id);
        //if the node is still on the stack and its low-link value is greater than that of its neighbour then update the low-link of current node
        if (onStack[temp] && (lows.find(head) == lows.end() || lows[head] > lows[temp])) lows[head] = lows[temp];
    }

    //if the low-link and id are the same then there is a strongly connected component
    if (ids[head] == lows[head]) {
        //start emptying the stack until the top node is the current one
        //in this loop you could also save in a vector of vectors all of the strongly connected components
        for (Vertex* node = s.top(); true; node = s.top()) {
            s.pop();
            onStack[node] = false;
            lows[node] = lows[head];
            if (node == head) break;
        }
        //increase the amount of strongly connected components
        amount++;
    }
}

int tarjan(std::vector<Vertex*>& g) {
    //setting up all of the parameters for the DFS calls
    int n = g.size();
    int amount = 0;
    int id = 0;
    std::unordered_map<Vertex*, int> ids;
    std::unordered_map<Vertex*, int> lows;
    std::unordered_map<Vertex*, bool> onStack;
    std::stack<Vertex*> s;
    //call DFS on all previously univisted nodes
    for (int i = 0; i<n; i++) {
        if (ids.find(g[i]) == ids.end()) dfs_t(g[i], lows, ids, amount, onStack, s, id);
    }
    return amount;
}
```
The `tarjan` function is called with only one argument, the vector of vertices representing the graph, and it returns the number of SCCs found. With some small adjustments, you can have it return the SCCs themselves. The time complexity of this algorithm is equal to that of a DFS so it is just $O(V+E)$ and its space complexity is just $O(V)$ where V is the number of vertices and E is the number of edges.
