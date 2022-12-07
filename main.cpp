//#include "cuda_runtime.h"
//#include "device_launch_parameters.h"
//#include "graph.cuh"
//#include <cstdio>
//#include <cuda.h>
//#include "test_1.cu"
//
//extern void run(void);
//
//int main(int argc, char** argv) {
//
//	Graph G;
//	int startVertex = atoi(argv[1]);
//	readGraph(G, argc, argv);
//
//	/*std::vector<int> distance;
//	std::vector<int> parent;
//	std::fill(distance.begin(), distance.end(), std::numeric_limits<int>::max());
//	std::fill(parent.begin(), parent.end(), std::numeric_limits<int>::max());*/
//
//	printf("Number of vertices %d\n", G.numVertices);
//	printf("Number of edges %d\n\n", G.numEdges);
//
//
//	//vectors for results
//	std::vector<int> resultVector(G.numVertices, std::numeric_limits<int>::max());
//	int N = G.numVertices;
//
//	bool* h_test;
//
//	cudaMalloc(&h_test, N * sizeof(bool));
//	for (int k = 0; k < N; k++) h_test[k] = true;
//
//	int* k_still_running = new int(1);
//	//cudaMalloc(&k_still_running, N * sizeof(bool));
//
//	int* still_running = new int(1);
//	//cudaMalloc(&still_running, N * sizeof(bool));
//
//
//	int BLOCKS = 1;
//	dim3 THREADS(N, N);
//
//	int* adj_length = &(*G.edgesSize.begin());
//	int* adj_list = &(*G.adjacencyList.begin());
//	int* adj_begin = &(*G.edgesOffset.begin());
//	int* result = &(*resultVector.begin());
//
//
//
//	while (*still_running) {
//		{
//			*still_running = 0;
//			*k_still_running = 0;
//			cudaMemcpy(&k_still_running, &h_test, sizeof(bool) * 1, cudaMemcpyHostToDevice);
//			int* args[] = { &N, adj_length,adj_list, adj_begin, result, still_running };
//			//kernel_cuda_simple <<<BLOCKS, THREADS>>> (N, adj_length,adj_list, adj_begin, result, still_running);
//			cudaLaunchKernel(&kernel_cuda_simple, G.numVertices / 1024 + 1, 1024, 1, args, 0, 0);
//			cudaLaunchKernel()
//			cudaMemcpy(still_running, k_still_running, sizeof(bool) * 1, cudaMemcpyDeviceToHost);
//		}
//	}
//
//	cudaThreadSynchronize();
//	cudaFree(still_running);
//	cudaFree(k_still_running);
//	cudaFree(h_test);
//	return 0;
//}