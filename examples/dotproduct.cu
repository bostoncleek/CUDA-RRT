#include <stdio.h>

#define imin(a,b) (a<b ? a:b);
#define sum_squares(x) (x*(x+1)*(2*x+1)/6)


const int N = 33*1024;
const int THREADS_PER_BLOCK = 256;
const int NUMBER_BLOCKS = 2;//imin(32, (N+THREADS_PER_BLOCK-1) / THREADS_PER_BLOCK);


__global__ void dot( float *a, float *b, float *c ) {

    printf("Hello frome block %d, thread %d\n", blockIdx.x, threadIdx.x);

    // data shared between threads in the same block
    // cache will contain each threads running sum
    __shared__ float cache[THREADS_PER_BLOCK];

    int tid = threadIdx.x + blockIdx.x * blockDim.x;
    int cacheIndex = threadIdx.x;

    // printf("Cache Index %d\n", cacheIndex);

    float temp = 0;
    while (tid < N) {
        temp += a[tid] * b[tid];
        tid += blockDim.x * gridDim.x;
    }

    // set the cache values
    cache[cacheIndex] = temp;

    // synchronize threads in this block
    __syncthreads();

    // for reductions, THREADS_PER_BLOCK must be a power of 2
    // because of the following code
    // each thread will sum two values in the cache and add the result
    // this is more efficient than having one thread do it all
    int i = blockDim.x/2;
    while (i != 0) {
        // printf("i %d\n", i);

        if (cacheIndex < i)
            cache[cacheIndex] += cache[cacheIndex + i];

        __syncthreads();
        i /= 2;
    }

    // each block writes to global memory
    // each entry in c contains the sum produced by a parallel block
    if (cacheIndex == 0)
        c[blockIdx.x] = cache[0];
}



int main() {

    float   *a, *b, c, *partial_c;
    float   *dev_a, *dev_b, *dev_partial_c;

    // allocate memory on the cpu side
    a = (float*)malloc( N*sizeof(float) );
    b = (float*)malloc( N*sizeof(float) );
    partial_c = (float*)malloc( NUMBER_BLOCKS*sizeof(float) );

    // allocate the memory on the GPU
    cudaMalloc( (void**)&dev_a, N*sizeof(float));
    cudaMalloc((void**)&dev_b, N*sizeof(float));
    cudaMalloc((void**)&dev_partial_c, NUMBER_BLOCKS*sizeof(float)) ;

    // fill in the host memory with data
    for (int i=0; i<N; i++) {
        a[i] = i;
        b[i] = i*2;
    }

    // copy the arrays 'a' and 'b' to the GPU
    cudaMemcpy(dev_a, a, N*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_b, b, N*sizeof(float), cudaMemcpyHostToDevice);

    dot<<< NUMBER_BLOCKS , THREADS_PER_BLOCK >>>( dev_a, dev_b, dev_partial_c );

    // copy the array 'c' back from the GPU to the CPU
    cudaMemcpy(partial_c,
               dev_partial_c,
               NUMBER_BLOCKS*sizeof(float),
               cudaMemcpyDeviceToHost);

    // finish up on the CPU side
    c = 0;
    for (int i=0; i < NUMBER_BLOCKS; i++) {
        c += partial_c[i];
    }

    printf( "Does GPU value %.6g = %.6g?\n", c,
             2 * sum_squares( (float)(N - 1) ) );

    // free memory on the gpu side
    cudaFree( dev_a );
    cudaFree( dev_b );
    cudaFree( dev_partial_c );

    // free memory on the cpu side
    free( a );
    free( b );
    free( partial_c );

    return 0;
}
