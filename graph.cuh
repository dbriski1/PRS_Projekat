#ifndef BFS_CUDA_GRAPH_H
#define BFS_CUDA_GRAPH_H

#include <vector>
#include <cstdio>
#include <cstdlib>

struct Graph {
    std::vector<long int> adjacencyList; // all edges
    std::vector<long int> edgesOffset; // offset to adjacencyList for every vertex
    std::vector<long int> edgesSize; //number of edges for every vertex
    long int numVertices = 0;
    long int numEdges = 0;
};

void readGraph(Graph& G);

#endif //BFS_CUDA_GRAPH_H