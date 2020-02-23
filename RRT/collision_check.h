#ifndef COLLISON_CHECK
#define COLLISON_CHECK



void obstacle_collision(float *cx, float *cy, float *r, float *q_new, uint8_t *d_obs_flag);



void copyToDeviceMemory(void* d, void* h, size_t size);

void copyToHostMemory(void* h, void* d, size_t size);

void* allocateDeviceMemory(size_t size);

void freeDeviceMemory(void* d);


#endif
