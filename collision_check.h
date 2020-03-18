#ifndef COLLISON_CHECK
#define COLLISON_CHECK


void bin_call(float3 *c, float3 *bins, uint32_t mem_size);

void collision_call_1(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag);

void collision_call_2(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag, int num_circles);

void collision_call_3(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag);

void copyToDeviceMemory(void* d, void* h, size_t size);

void copyToHostMemory(void* h, void* d, size_t size);

void* allocateDeviceMemory(size_t size);

void freeDeviceMemory(void* d);


#endif
