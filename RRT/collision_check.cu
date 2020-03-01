#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cutil.h>
#include "collision_check.h"

#include "collision_check.h"


__global__ void kernelSanders1(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag);

__device__ float distance(float cx, float cy, float *qnew);

__device__ float distToCenter(float cx, float cy, float u, float *qnew, float *qnear);

__device__ float composeU(float cx, float cy, float *qnew, float *qnear);




__global__ void kernelSanders1(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *collision_flag)
{


  const int tid = threadIdx.x;

  __shared__ uint32_t flag;
  flag = 0;

  const float c_x = cx[tid];
  const float c_y = cy[tid];
  const float c_r = r[tid];

  const float u = composeU(c_x, c_y, q_new, q_near);
  const float dist_to_ray = distToCenter(c_x, c_y, u, q_new, q_near);
  const float dist_to_q_new = distance(c_x, c_y, q_new);


  // shortest distance to your ray exists in the circle
  if ((dist_to_ray < c_r) && (u < 1) && (u > 0))
  {
    //SET FLAG TO TRUE SHORTEST POINT ON LINE IN CIRLE
    atomicAdd(&flag, 1);
  }


  if(dist_to_q_new < c_r)
  {
    //SET THE FLAG NEW POINT IN CIRCLE
    atomicAdd(&flag, 1);
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
  float dx = cx - qnew[0];
  float dy = cy - qnew[1];
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
  float num = (cx-qnew[0])*(qnear[0]-qnew[0]) + (cy-qnew[1])*(qnear[1]-qnew[1]);
  float denom = (qnear[0]-qnew[0])*(qnear[0]-qnew[0]) + (qnear[1]-qnew[1])*(qnear[1]-qnew[1]);
  return num / denom;
}





void collision_call(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag)
{
  // set flag to 0
  cudaMemset(flag, 0, sizeof(uint32_t));


  dim3 dimGrid(1);
  dim3 dimBlock(1024);

  kernelSanders1<<<dimGrid, dimBlock>>>(cx, cy, r, q_new, q_near, flag);

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
