#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cutil.h>


__global__ void obstacleKernel(uint32_t *x, uint32_t *y, uint32_t *r, uint32_t *q_new);



__global__ void obstacleKernel(uint32_t *x, uint32_t *y, uint32_t *r, uint32_t *q_new)
{


}



void obstacle_collision(uint32_t *x, uint32_t *y, uint32_t *r, uint32_t *q_new)
{




}





void copyToDeviceMemory(void* d, void* h, size_t size)
{
	cudaMemcpy(d, h, size, cudaMemcpyHostToDevice);
}


void copyToHostMemory(void* h, void* d, size_t size)
{
	cudaMemcpy(h, d, size, cudaMemcpyDeviceToHost);
}


void* allocateDeviceMemory(size_t size)
{
  void *ptr;
  cudaMalloc(&ptr, size);
  return ptr;
}


void freeDeviceMemory(void* d)
{
	cudaFree(d);
}
