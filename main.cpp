//#include <iostream>
//#include <vector>
//#include <list>
//#include <fstream>
//#include <string>
//
//class Graph
//{
//    int V;
//
//    std::vector<std::list<int>> adj;
//public:
//    Graph();
//
//    void BFS(int s);
//
//    void readGraph();
//
//    int getNumberVertices();
//
//    friend void convert_to_numeric(std::string value, int& a, int& b);
//};
//int Graph::getNumberVertices() {
//    return V;
//}
//
//Graph::Graph() {
//
//}
//
//void Graph::BFS(int s)
//{
//    std::vector<bool> visited;
//    visited.resize(V, false);
//
//    
//    std::list<int> queue;
//
//    visited[s] = true;
//    queue.push_back(s);
//
//    while (!queue.empty())
//    {
//       
//        s = queue.front();
//        std::cout << s << " ";
//        queue.pop_front();
//
//        for (auto adjecent : adj[s])
//        {
//            if (!visited[adjecent])
//            {
//                visited[adjecent] = true;
//                queue.push_back(adjecent);
//            }
//        }
//    }
//}
//
//void Graph::readGraph() {
//    int n;
//    int m;
//    bool console_input = false;
//
//
//    std::string n_vertices_n_edges;
//
//
//    std::ifstream MyReadFile("p2p-Gnutella08.txt");
//
//    if (console_input) {
//        scanf("%d %d", &n, &m);
//    }
//    else {
//        std::getline(MyReadFile, n_vertices_n_edges);
//        convert_to_numeric(n_vertices_n_edges, n, m);
//    }
//
//    std::string edge;
//    adj.resize(n);
//    V = n;
//    //std::vector<std::vector<int> > adjecancyLists(n);
//
//    for (int i = 0; i < m; i++) {
//        int u, v;
//        if (console_input) {
//            scanf("%d %d", &u, &v);
//        }
//        else {
//            getline(MyReadFile, edge);
//            convert_to_numeric(edge, u, v);
//        }
//        adj[u].push_back(v);
//    }
//
//    MyReadFile.close();
//}
//
//void convert_to_numeric(std::string value, int& a, int& b) {
//    std::string delimiter = "\t";
//
//    size_t pos = 0;
//    std::string token;
//    pos = value.find(delimiter);
//    token = value.substr(0, pos);
//    value.erase(0, pos + delimiter.length());
//    a = stoi(token);
//    b = stoi(value);
//}
//
//
//
//int main() {
//
//	Graph G;
//	G.readGraph();
//
//	printf("Number of vertices %d\n", G.getNumberVertices());
//
//
//	//vectors for results
//	std::vector<int> resultVector(G.getNumberVertices(), std::numeric_limits<int>::max());
//	int N = G.getNumberVertices();
//	G.BFS(0);
//
//
//	return 0;
//}