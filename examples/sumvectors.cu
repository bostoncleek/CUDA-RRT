#include <stdio.h>
#include <time.h>

// Number of blocks to Launch
#define N 500000


// Called on host and executed on device
__global__ void addGPU(int *a, int *b, int *c)
{
  int tid = blockIdx.x;
  if(tid < N)
    c[tid] = a[tid] + b[tid];
}



// Run on CPU
void addCPU(int *a, int *b, int *c)
{
  int tid = 0;
  while(tid < N){
    c[tid] = a[tid] + b[tid];
    tid += 1;
  }
}

int main()
{
  int startTime, endTime;
  double execTime;

  int a[N], b[N], c[N];         // host variables
  int *dev_a, *dev_b, *dev_c;   // device variables

   // fill arrays 'a' and 'b' on CPU
   for(int i = 0; i < N; i++){
     a[i] = -i;
     b[i] = i * i;
   }

  // First test on CPU
  startTime = clock();
  addCPU(a, b, c);
  endTime = clock();

  execTime = (double)(endTime-startTime)/CLOCKS_PER_SEC;
  printf("Execution time on CPU: %f (ms)\n", execTime*1000);


  // Test on GPU
  // Allocate memory on GPU
  cudaMalloc( (void**)&dev_a, N * sizeof(int) );
  cudaMalloc( (void**)&dev_b, N * sizeof(int) );
  cudaMalloc( (void**)&dev_c, N * sizeof(int) );

  // Copy arrays 'a' and 'b' to the GPU
  cudaMemcpy( dev_a, a, N * sizeof(int), cudaMemcpyHostToDevice );
  cudaMemcpy( dev_b, b, N * sizeof(int), cudaMemcpyHostToDevice );
  cudaMemcpy( dev_c, c, N * sizeof(int), cudaMemcpyHostToDevice );

  // Launch kernel ofn GPU
  // paramets <<N,M>
  // 1) N is the number of parallel blocks
  // there are N copies of the kernel running in parallel
  // a collection of parallel blocks is a grid
  // 2) M is the number of threads per block
  startTime = clock();
  addGPU<<<N,1>>>( dev_a, dev_b, dev_c );
  endTime = clock();

  execTime = (double)(endTime-startTime)/CLOCKS_PER_SEC;
  printf("Execution time on GPU: %f (ms)\n", execTime*1000);


  // Copy the array 'c' back from the GPU to the CPU
  cudaMemcpy( c, dev_c, N * sizeof(int), cudaMemcpyDeviceToHost );

  cudaFree( dev_a );
  cudaFree( dev_b );
  cudaFree( dev_c );

  return 0;
}














//
