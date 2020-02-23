#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cutil.h>


__global__ void obstacleKernel(float *cx, float *cy, float *r, float *q_new, uint8_t *d_obs_flag);

__device__ double distance(float cx, float cy, float *q);



__global__ void obstacleKernel(float *cx, float *cy, float *r, float *q_new, uint8_t *d_obs_flag)
{
  // TODO; copy over epsilon


  const int tid = threadIdx.x;

  __shared__ uint8_t flag;
  flag = 0;

  double d = distance(cx[tid], cy[tid], q_new);

  if (d < r[tid])
  {
    flag = 1;
  }

  __syncthreads();


  // have one thread write result to global memory
  if (tid == 0)
  {
    if (flag)
    {
      *d_obs_flag = 1;
    }

    else
    {
      *d_obs_flag = 0;
    }
  }
}



__device__ double distance(float cx, float cy, float *q)
{
  float dx = cx - q[0];
  float dy = cy - q[1];

  return sqrt(dx*dx + dy*dy);
}





void obstacle_collision(float *cx, float *cy, float *r, float *q_new, uint8_t *d_obs_flag)
{
  // set flag to 0
  cudaMemset(d_obs_flag, 0, sizeof(uint8_t));


  dim3 dimGrid(1);
  dim3 dimBlock(5);

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
