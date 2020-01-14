#include <stdio.h>

__global__ void add(int a, int b, int *c)
{
  *c = a + b;
}

int main()
{
  // Allocate memory on device
  // Do NOT dereference to pointer returned by cudaMalloc
  // Arguements:
  // 1) pointer to pointer to hold adress of newlly allocate memory
  // 2) size of allocation

  int c;
  int *dev_c;

  cudaMalloc((void**)&dev_c, sizeof(int)) ;

  // Launch kernel on device
  add<<<1,1>>>(2, 7, dev_c);

  // Access device memory
  cudaMemcpy(&c, dev_c, sizeof(int), cudaMemcpyDeviceToHost);

  printf("2 + 7 = %d\n", c);
  // printf("size of int: %lu\n", sizeof(int));

  // Free memroy allocated on device
  cudaFree(dev_c);

  return 0;
}
