#ifndef COLLISON_CHECK
#define COLLISON_CHECK

const float EPSILON =.5;



void collision_call(float *cx, float *cy, float *r, float *q_new, float *q_near, uint32_t *flag);

void copyToDeviceMemory(void* d, void* h, size_t size);

void copyToHostMemory(void* h, void* d, size_t size);

void* allocateDeviceMemory(size_t size);

void freeDeviceMemory(void* d);


#endif
