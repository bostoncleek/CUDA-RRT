#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <cutil.h>
#include "collision_check.h"

#include "collision_check.h"



// device global variables
__device__ uint32_t g_max_circles_cell = 100;
__device__ uint32_t nth_cirlce[100];
__device__ uint32_t g_xsize = 100;
__device__ uint32_t g_ysize = 100;

__device__ float g_resolution = 1.0;
__device__ float g_xmin = 0.0;
__device__ float g_xmax = 100.0;
__device__ float g_ymin = 0.0;
__device__ float g_ymax = 100.0;


// kernel declarations
__global__ void binCircles(float3 *c, float3 *bins);

__global__ void kernelSanders1(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *collision_flag);

__global__ void kernelSanders2(float3 *c, float *q_new, float *q_near, uint32_t *collision_flag);



// device functions
__device__ float distance(float cx, float cy, float *qnew);

__device__ float distToCenter(float cx, float cy, float u, float *qnew, float *qnear);

__device__ float composeU(float cx, float cy, float *qnew, float *qnear);

__device__ int world2RowMajor(float x, float y);



// kernels
__global__ void binCircles(float3 *c, float3 *bins)
{
  const int tid = threadIdx.x + blockDim.x * blockIdx.x;

  const float c_x = c[tid].x;
  const float c_y = c[tid].y;
  const float c_r = c[tid].z;

  __syncthreads();


  int center = world2RowMajor(c_x, c_y);
  int top  = world2RowMajor(c_x, c_y + c_r);
  int left  = world2RowMajor(c_x - c_r, c_y);
  int bottom  = world2RowMajor(c_x, c_y - c_r);
  int right  = world2RowMajor(c_x + c_r, c_y);

  __syncthreads();

  // printf("center: %d\n", center);

  int coords[] = {top, left, bottom, right};
  int uniq[] = {center, -1, -1, -1};
  uint iterator = 1;

  for(int i = 0; i < 4; i++) //iterate through top left right bottom
  {
    for(int j = 0; j < iterator; j++)
    {
      if (coords[i] != uniq[j] && coords[i] != -1)
      {
        iterator++;
        uniq[iterator] = coords[i];
      }
    }
  }

  __syncthreads();


  for(int i = 0; i < iterator; i++)
  {
  // printf("uniq[iterator]: %u\n", uniq[iterator]);

   uint bin_col = atomicInc(&nth_cirlce[uniq[iterator]], g_max_circles_cell);
   uint bin_index = uniq[iterator] * g_max_circles_cell + bin_col;
   printf("bin_index: %u\n", bin_index);

   bins[bin_index] = c[tid];
   // printf("[x: %f y: %f r: %f] \n", bins[bin_index].x, bins[bin_index].y, bins[bin_index].z);

  }

  __syncthreads();

}



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



__global__ void kernelSanders2(float3 *c, float *q_new, float *q_near, uint32_t *collision_flag)
{


  const int tid = threadIdx.x + blockDim.x * blockIdx.x;

  __shared__ uint32_t flag;
  flag = 0;

  const float c_x = c[tid].x;
  const float c_y = c[tid].y;
  const float c_r = c[tid].z;

  const float u = composeU(c_x, c_y, q_new, q_near);
  const float dist_to_ray = distToCenter(c_x, c_y, u, q_new, q_near);
  const float dist_to_q_new = distance(c_x, c_y, q_new);


  // shortest distance to your ray exists in the circle
  if ((dist_to_ray < c_r) && (u < 1) && (u > 0))
  {
    //SET FLAG TO TRUE SHORTEST POINT ON LINE IN CIRLE
    atomicAdd(&flag, 1);
    // *collision_flag = 1;
    // return;
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


__device__ int world2RowMajor(float x, float y)
{
  if (!(x >= g_xmin && x <= g_xmax))
  {
    return -1;
  }

  if (!(y >= g_ymin and y <= g_ymax))
  {
    return -1;
  }

  int i = std::floor((x - g_xmin) / g_resolution);
  int j = std::floor((y - g_ymin) / g_resolution);



  if (i == g_xsize)
  {
    i--;
  }

  if (j == g_ysize)
  {
    j--;
  }

  return i * g_xsize + j;
}




void bin_call(float3 *c, float3 *bins, uint32_t mem_size)
{
  // cudaMemset(bins, 0.0, mem_size*sizeof(float3));

  dim3 dimGrid(16);
  dim3 dimBlock(512);

  binCircles<<<dimGrid, dimBlock>>>(c, bins);

  cudaThreadSynchronize();
}



void collision_call_1(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag)
{
  // set flag to 0
  cudaMemset(flag, 0, sizeof(uint32_t));


  dim3 dimGrid(16);
  dim3 dimBlock(512);

  kernelSanders1<<<dimGrid, dimBlock>>>(cx, cy, r, q_new, q_near, flag);

  cudaThreadSynchronize();
}



void collision_call_2(float3 *c, float *q_new, float *q_near, uint32_t *flag)
{
  // set flag to 0
  cudaMemset(flag, 0, sizeof(uint32_t));


  dim3 dimGrid(16);
  dim3 dimBlock(512);

  kernelSanders2<<<dimGrid, dimBlock>>>(c, q_new, q_near, flag);

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
