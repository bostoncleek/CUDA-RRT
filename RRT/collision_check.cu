#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cutil.h>

#include "collision_check.h"


#define EPSILON 1

__global__ void obstacleKernel(float *cx, float *cy, float *r, float *q_new, uint32_t *d_obs_flag);

__device__ float distance(float cx, float cy, float *q);

__global__ void obstacleKernel(float *cx, float *cy, float *r, float *q_new, uint32_t *d_obs_flag)
{


  const int tid = threadIdx.x;

  __shared__ uint32_t flag;
  flag = 0;


  float d = distance(cx[tid], cy[tid], q_new);
  // printf("%f %f %f\n", cx[tid], cy[tid], r[tid]);

  // if (d < r[tid] + EPSILON)
  // {
  //   printf("collides !!!!!!!!!!!\n");
  //   *d_obs_flag = 1;
  // }


  if (d < r[tid] + EPSILON)
  {
    atomicAdd(&flag, 1);
    // printf("collides !!!!!!!!!!!\n");
    // flag = 1;
  }

  __syncthreads();


  // have one thread write result to global memory
  if (tid == 0)
  {
    if (flag > 0)
    {
      *d_obs_flag = 1;
    }

    else
    {
      *d_obs_flag = 0;
    }
  }
}



__device__ float distance(float cx, float cy, float *q)
{
  float dx = cx - q[0];
  float dy = cy - q[1];

  return sqrt(dx*dx + dy*dy);
}





void obstacle_collision(float *cx, float *cy, float *r, float *q_new, uint32_t *d_obs_flag)
{
  // set flag to 0
  cudaMemset(d_obs_flag, 0, sizeof(uint32_t));


  dim3 dimGrid(1);
  dim3 dimBlock(1024);

  obstacleKernel<<<dimGrid, dimBlock>>>(cx, cy, r, q_new, d_obs_flag);

  cudaThreadSynchronize();
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
