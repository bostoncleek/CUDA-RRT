#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cutil.h>
#include "collision_check.h"

#include "collision_check.h"


#define EPSILON 1

__global__ void kernelSanders(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag);

__device__ float distance(float cx, float cy, float *qnew);

__device__ float distToCenter(float cx, float cy, float u, float *qnew, float *qnear);

__device__ float composeU(float cx, float cy, float *qnew, float *qnear);




__global__ void kernelSanders(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *collision_flag)
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
      *collision_flag = 1;
    }

    else
    {
      *collision_flag = 0;
    }
  }
}



__device__ float distance(float cx, float cy, float *qnew)
{
  float dx = cx - q[0];
  float dy = cy - q[1];
  return sqrt(dx*dx + dy*dy);
}


__device__ float distToCenter(float cx, float cy, float u, float *qnew, float *qnear)
{
  float x = qnew[0] + u*(qnear[0]-qnew[0]);
  float y = qnew[1] + u*(qnear[1]-qnew[1]);
  float p[2] = {x, y};

  return distance(cx, cy, p);
}


__device__ float composeU(float cx, float cy, float *qnew, float *qnear)
{
  float num = (cx-qnew[0])(qnear[0]-qnew[0]) + (cy-qnew[1])*(qnear[1]-qnew[1]);
  float denom = (qnear[0]-qnew[0])*(qnear[0]-qnew[0]) + (qnear[1]-qnew[1])*(qnear[1]-qnew[1]);
  return num / denom;
}



void collision_call(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag)
{
  // set flag to 0
  cudaMemset(flag, 0, sizeof(uint32_t));


  dim3 dimGrid(1);
  dim3 dimBlock(1024);

  kernelSanders<<<dimGrid, dimBlock>>>(cx, cy, r, q_new, q_near, flag);

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
