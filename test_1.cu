
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "graph.cuh"
#include <stdio.h>

__global__ void kernel_cuda_simple(
	int num_vertices,
	int* v_adj_length,
	int* v_adj_list,
	int* v_adj_begin,
	int* result,
	bool* still_running
	)
 {
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	int num_threads = blockDim.x * gridDim.x;
	

	for (int v = 0; v < num_vertices; v += num_threads)
	{
		int vertex = v + tid;
		//printf("%d ", num_vertices);
		if (vertex < num_vertices)
		{
			
			
			//printf("Velicina adjacency matrice za vertex %d: %d\n", vertex, v_adj_length[vertex]);
			for (int n = 0; n < v_adj_length[vertex]; n++)
			{

				int neighbor = v_adj_list[v_adj_begin[vertex] + n];
				//printf("Susjed cvora %d je: %d\n",vertex, neighbor);
					if (result[neighbor] > result[vertex] + 1)
					{
						printf("%d %d\n", result[neighbor], result[vertex] + 1);
						result[neighbor] = result[vertex] + 1;
						* still_running = 1;
						printf("Rezultat za izvorni cvor %d je %d, a za ciljni cvor %d je %d\n", vertex, result[vertex], neighbor, result[neighbor]);
					}
				
			}
		}
	}
	printf("\n");
}

 void run(Graph G, std::vector<int> &resultVector)
 { 
	 
	 int N = G.numVertices;
	 
	 int * d_adj_length, * d_adj_list, * d_adj_begin, *d_result;
	 bool* d_still_running;
	 //bool* h_test = new bool(true);
	 

	 /*cudaMalloc(&h_test, N * sizeof(bool));
	 for (int k = 0; k < N; k++) h_test[k] = true;*/

	 bool* k_still_running = new bool(true);
	 //cudaMalloc(&k_still_running, N * sizeof(bool));

	 bool* still_running = (bool*)malloc(1 * sizeof(bool));;
	 //cudaMalloc(&still_running, N * sizeof(bool));


	 int BLOCKS = 1;
	 dim3 THREADS(N, 1, 1);

	 int* adj_length = &(*G.edgesSize.begin());
	 int* adj_list = &(*G.adjacencyList.begin());
	 int* adj_begin = &(*G.edgesOffset.begin());
	 int* result = &(*resultVector.begin());

	 cudaMalloc(&d_still_running, 1 * sizeof(bool));
	 cudaMalloc(&d_adj_length, G.edgesSize.size() * sizeof(int));
	 cudaMalloc(&d_adj_list, G.adjacencyList.size() * sizeof(int));
	 cudaMalloc(&d_adj_begin, G.edgesOffset.size() * sizeof(int));
	 cudaMalloc(&d_result, N * sizeof(int));

	 

	 //printf("%d ", adj_begin[0]);

	 while (*still_running) {
		 {
			 *still_running = false;
			 *k_still_running = false;
			 cudaMemcpy(d_still_running, still_running, sizeof(bool) * 1, cudaMemcpyHostToDevice);
			 cudaMemcpy(d_adj_length, adj_length, G.edgesSize.size() * sizeof(int), cudaMemcpyHostToDevice);
			 cudaMemcpy(d_adj_list, adj_list, G.adjacencyList.size() * sizeof(int), cudaMemcpyHostToDevice);
			 cudaMemcpy(d_adj_begin, adj_begin, G.edgesOffset.size() * sizeof(int), cudaMemcpyHostToDevice);
			 cudaMemcpy(d_result, result, N * sizeof(int), cudaMemcpyHostToDevice);
			 //cudaMemcpy(d_y, y, N * sizeof(int), cudaMemcpyHostToDevice);
			 
			 for (int i = 0; i < N; i++) {
				 printf("%d ", result[i]);
			 }
			 printf("\n");

			 kernel_cuda_simple <<<BLOCKS, THREADS>>> (N, d_adj_length, d_adj_list, d_adj_begin, d_result, d_still_running);
			 //cudaLaunchKernel(kernel_cuda_simple, G.numVertices / 1024 + 1, 1024, 1, args, 0, 0);
			 cudaMemcpy(still_running, d_still_running, sizeof(bool) * 1, cudaMemcpyDeviceToHost);
			 cudaMemcpy(adj_length, d_adj_length, G.edgesSize.size() * sizeof(int), cudaMemcpyDeviceToHost);
			 cudaMemcpy(adj_list, d_adj_list, G.adjacencyList.size() * sizeof(int), cudaMemcpyDeviceToHost);
			 cudaMemcpy(adj_begin, d_adj_begin, G.edgesOffset.size() * sizeof(int), cudaMemcpyDeviceToHost);
			 cudaMemcpy(result, d_result, N * sizeof(int), cudaMemcpyDeviceToHost);
		 }
	 }
	
	cudaThreadSynchronize();
	cudaFree(still_running);
	cudaFree(k_still_running);

 }

 int main() {
	 Graph G;
	 //int startVertex = atoi(argv[1]);
	 readGraph(G);

	 /*std::vector<int> distance;
	 std::vector<int> parent;
	 std::fill(distance.begin(), distance.end(), std::numeric_limits<int>::max());
	 std::fill(parent.begin(), parent.end(), std::numeric_limits<int>::max());*/

	 printf("Number of vertices %d\n", G.numVertices);
	 printf("Number of edges %d\n\n", G.numEdges);


	 //vectors for results
	 std::vector<int> resultVector(G.numVertices, std::numeric_limits<int>::max());
	 resultVector[0] = 0;


	 run(G, resultVector);
	 for (int i = 0; i < resultVector.size(); i++) {
		 printf("%d  ", resultVector[i]);
	 }
	 return 0;
 }