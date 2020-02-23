#ifndef COLLISON_CHECK
#define COLLISON_CHECK



void obstacle_collision(uint32_t *x, uint32_t *y, uint32_t *r, uint32_t *q_new);



void copyToDeviceMemory(void* d, void* h, size_t size);

void copyToHostMemory(void* h, void* d, size_t size);

void* allocateDeviceMemory(size_t size);

void freeDeviceMemory(void* d);


#endif
