
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "graph.cuh"
#include <stdio.h>
#include <chrono>


//  Vertex-centric Implementation Explicit Iteration Counter
__global__ void kernel_cuda_simple(
	long int num_vertices,
	long int level,
	long int* v_adj_length,
	long int* v_adj_list,
	long int* v_adj_begin,
	long int* result,
	bool* still_running
	)
 {
	int tid = blockIdx.x * blockDim.x + threadIdx.x;
	int num_threads = blockDim.x * gridDim.x;
	//printf("Broj tid: %d\n", tid);

	for (long int v = 0; v < num_vertices; v += num_threads)
	{
		long int vertex = v + tid;
		//printf("Trenutni thread je %d, a vertex je %d\n", tid, vertex);
		if (result[vertex] == level && vertex < num_vertices)
		{
			//printf("Usao u vertex broj: %d\n", vertex);
			
			//printf("Velicina adjacency matrice za vertex %d: %d\n", vertex, v_adj_length[vertex]);
			for (long int n = 0; n < v_adj_length[vertex]; n++)
			{

				long int neighbor = v_adj_list[v_adj_begin[vertex] + n];
				//printf("Susjed cvora %d je: %d\n",vertex, neighbor);
					if (result[neighbor] > result[vertex] + 1)
					{
						//printf("%d %d\n", result[neighbor], result[vertex] + 1);
						result[neighbor] = result[vertex] + 1;
						* still_running = 1;
						//printf("Rezultat za izvorni cvor %d je %d, a za ciljni cvor %d je %d\n", vertex, result[vertex], neighbor, result[neighbor]);
					}
				
			}
		}
	}
	//printf("\n");
}

__global__ void kernel_cuda_optimized(
	long int num_vertices,
	long int level,
	long int* v_adj_length,
	long int* v_adj_list,
	long int* v_adj_begin,
	long int* result,
	bool* still_running
)
{
	long int tid = blockIdx.x * blockDim.x + threadIdx.x;
	if (tid < num_vertices && result[tid] == level) {
		long int u = tid;
		for (long int i = v_adj_begin[u]; i < v_adj_begin[u] + v_adj_length[u]; i++) {
			long int v = v_adj_list[i];
			if (level + 1 < result[v]) {
				result[v] = level + 1;
				/*d_parent[v] = i;*/
				*still_running = 1;
			}
		}
	}
	//printf("\n");
}

 void run_simple(Graph G, std::vector<long int> &resultVector)
 { 
	 
	 
	 long int N = G.numVertices;
	 
	 long int * d_adj_length, * d_adj_list, * d_adj_begin, *d_result, d_level;
	 bool* d_still_running;
	 
	 //bool* h_test = new bool(true);
	 

	 /*cudaMalloc(&h_test, N * sizeof(bool));
	 for (int k = 0; k < N; k++) h_test[k] = true;*/

	 bool* k_still_running = new bool(true);
	 //cudaMalloc(&k_still_running, N * sizeof(bool));

	 bool* still_running = (bool*)malloc(1 * sizeof(bool));;
	 //cudaMalloc(&still_running, N * sizeof(bool));


	 int BLOCKS = 8;
	 dim3 THREADS(128, 1, 1);

	 long int* adj_length = &(*G.edgesSize.begin());
	 long int* adj_list = &(*G.adjacencyList.begin());
	 long int* adj_begin = &(*G.edgesOffset.begin());
	 long int* result = &(*resultVector.begin());

	 cudaMalloc(&d_still_running, 1 * sizeof(bool));
	 cudaMalloc(&d_adj_length, G.edgesSize.size() * sizeof(long int));
	 cudaMalloc(&d_adj_list, G.adjacencyList.size() * sizeof(long int));
	 cudaMalloc(&d_adj_begin, G.edgesOffset.size() * sizeof(long int));
	 cudaMalloc(&d_result, N * sizeof(long int));
	 

	 /*printf("Velicina edges size: %d\n", G.edgesSize.size());
	 printf("Velicina adjacency list: %d\n", G.adjacencyList.size());
	 printf("Velicina edges offset: %d\n", G.edgesOffset.size());

	 printf("Starting simple parallel bfs.\n");*/
	 
	 long int level = 0;
	 cudaMemcpy(d_adj_length, adj_length, G.edgesSize.size() * sizeof(long int), cudaMemcpyHostToDevice);
	 cudaMemcpy(d_adj_list, adj_list, G.adjacencyList.size() * sizeof(long int), cudaMemcpyHostToDevice);
	 cudaMemcpy(d_adj_begin, adj_begin, G.edgesOffset.size() * sizeof(long int), cudaMemcpyHostToDevice);
	 cudaMemcpy(d_result, result, N * sizeof(long int), cudaMemcpyHostToDevice);

	 auto start = std::chrono::steady_clock::now();
	 while (*still_running) {
		 {
			 *still_running = false;
			 cudaMemcpy(d_still_running, still_running, sizeof(bool) * 1, cudaMemcpyHostToDevice);
			 

			 kernel_cuda_simple <<<BLOCKS, THREADS>>> (N, level, d_adj_length, d_adj_list, d_adj_begin, d_result, d_still_running);
			 cudaMemcpy(still_running, d_still_running, sizeof(bool) * 1, cudaMemcpyDeviceToHost);

			 level++;
			 
		 }
	 }
	 cudaMemcpy(result, d_result, N * sizeof(long int), cudaMemcpyDeviceToHost);


	auto end = std::chrono::steady_clock::now();
	long duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

	cudaDeviceSynchronize();
	cudaFree(still_running);
	cudaFree(k_still_running);
	cudaFree(d_adj_length);
	cudaFree(d_adj_list);
	cudaFree(d_adj_begin);
	cudaFree(d_result);
	printf("Elapsed time in microseconds for simple BFS: %li ms.\n", duration);
 }

 void run_optimized(Graph G, std::vector<long int>& resultVector)
 {


	 long int N = G.numVertices;

	 long int* d_adj_length, * d_adj_list, * d_adj_begin, * d_result, d_level;
	 bool* d_still_running;

	 //bool* h_test = new bool(true);


	 /*cudaMalloc(&h_test, N * sizeof(bool));
	 for (int k = 0; k < N; k++) h_test[k] = true;*/

	 bool* k_still_running = new bool(true);
	 //cudaMalloc(&k_still_running, N * sizeof(bool));

	 bool* still_running = (bool*)malloc(1 * sizeof(bool));;
	 //cudaMalloc(&still_running, N * sizeof(bool));


	 int BLOCKS = 8;
	 dim3 THREADS(128, 1, 1);

	 long int* adj_length = &(*G.edgesSize.begin());
	 long int* adj_list = &(*G.adjacencyList.begin());
	 long int* adj_begin = &(*G.edgesOffset.begin());
	 long int* result = &(*resultVector.begin());

	 cudaMalloc(&d_still_running, 1 * sizeof(bool));
	 cudaMalloc(&d_adj_length, G.edgesSize.size() * sizeof(long int));
	 cudaMalloc(&d_adj_list, G.adjacencyList.size() * sizeof(long int));
	 cudaMalloc(&d_adj_begin, G.edgesOffset.size() * sizeof(long int));
	 cudaMalloc(&d_result, N * sizeof(long int));


	 /*printf("Velicina edges size: %d\n", G.edgesSize.size());
	 printf("Velicina adjacency list: %d\n", G.adjacencyList.size());
	 printf("Velicina edges offset: %d\n", G.edgesOffset.size());

	 printf("Starting simple parallel bfs.\n");*/

	 long int level = 0;
	 cudaMemcpy(d_adj_length, adj_length, G.edgesSize.size() * sizeof(long int), cudaMemcpyHostToDevice);
	 cudaMemcpy(d_adj_list, adj_list, G.adjacencyList.size() * sizeof(long int), cudaMemcpyHostToDevice);
	 cudaMemcpy(d_adj_begin, adj_begin, G.edgesOffset.size() * sizeof(long int), cudaMemcpyHostToDevice);
	 cudaMemcpy(d_result, result, N * sizeof(long int), cudaMemcpyHostToDevice);

	 auto start = std::chrono::steady_clock::now();
	 while (*still_running) {
		 {
			 *still_running = false;
			 cudaMemcpy(d_still_running, still_running, sizeof(bool) * 1, cudaMemcpyHostToDevice);


			 kernel_cuda_optimized << <BLOCKS, THREADS >> > (N, level, d_adj_length, d_adj_list, d_adj_begin, d_result, d_still_running);
			 cudaMemcpy(still_running, d_still_running, sizeof(bool) * 1, cudaMemcpyDeviceToHost);

			 level++;

		 }
	 }
	 cudaMemcpy(result, d_result, N * sizeof(long int), cudaMemcpyDeviceToHost);


	 auto end = std::chrono::steady_clock::now();
	 long duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

	 cudaDeviceSynchronize();
	 cudaFree(still_running);
	 cudaFree(k_still_running);
	 cudaFree(d_adj_length);
	 cudaFree(d_adj_list);
	 cudaFree(d_adj_begin);
	 cudaFree(d_result);
	 printf("Elapsed time in microseconds for optimized BFS: %li ms.\n", duration);
 }


 int main() {
	 Graph G;
	 readGraph(G);

	 printf("Number of vertices %d\n", G.numVertices);
	 printf("Number of edges %d\n\n", G.numEdges);

	 std::vector<long> resultVector_simple(G.numVertices, std::numeric_limits<long>::max());
	 resultVector_simple[0] = 0;


	 run_simple(G, resultVector_simple);
	 for (long i = 0; i < resultVector_simple.size(); i++) {
		 //printf("%d %d  \n",i, resultVector_simple[i]);
	 }
	 // Result for p2p-Gnutella08.txt is around 2300 microseconds

	 std::vector<long> resultVector_optimized(G.numVertices, std::numeric_limits<long>::max());
	 resultVector_optimized[0] = 0;


	 run_optimized(G, resultVector_optimized);
	 for (long i = 0; i < resultVector_optimized.size(); i++) {
		 //printf("%d %d  \n", i, resultVector_optimized[i]);
	 }
	 // Result for p2p-Gnutella08.txt is around 1200 microseconds

	 return 0;
 }