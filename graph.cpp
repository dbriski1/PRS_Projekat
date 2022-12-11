#include <ctime>
#include "graph.cuh"
#include <fstream>
#include <iostream>
#include <string>

void convert_to_numeric(std::string value, int& a, int& b) {
    std::string delimiter = "\t";

    size_t pos = 0;
    std::string token;
    pos = value.find(delimiter);
    token = value.substr(0, pos);
    value.erase(0, pos + delimiter.length());
    a = stoi(token);
    b = stoi(value);
}

void readGraph(Graph& G) {
    int n;
    int m;
    bool console_input = false;

    
    std::string n_vertices_n_edges;

    
    std::ifstream MyReadFile("p2p-Gnutella08.txt");
    
    

   

    //If no arguments then read graph from stdin
    //bool fromStdin = argc <= 2;
    

    if (console_input) {
        scanf("%d %d", &n, &m);
    }
    else {
        std::getline(MyReadFile, n_vertices_n_edges);
        convert_to_numeric(n_vertices_n_edges, n, m);
    }
   
    /*else {
        srand(12345);
        n = atoi(argv[2]);
        m = atoi(argv[3]);
    }*/
    std::string edge;
    std::vector<std::vector<int> > adjecancyLists(n);
    
    for (int i = 0; i < m; i++) {
        int u, v;
        if (console_input) {
            scanf("%d %d", &u, &v);
        }
        else {
            getline(MyReadFile, edge);
            convert_to_numeric(edge, u, v);
        }
        adjecancyLists[u].push_back(v);
        /*else {
            u = rand() % n;
            v = rand() % n;
            adjecancyLists[u].push_back(v);
            adjecancyLists[v].push_back(u);
        }*/
    }

    MyReadFile.close();

    for (int i = 0; i < n; i++) {
        G.edgesOffset.push_back(G.adjacencyList.size());
        G.edgesSize.push_back(adjecancyLists[i].size());
        for (auto& edge : adjecancyLists[i]) {
            G.adjacencyList.push_back(edge);
        }
    }

    G.numVertices = n;
    G.numEdges = G.adjacencyList.size();
}