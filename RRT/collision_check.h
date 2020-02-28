#ifndef COLLISON_CHECK
#define COLLISON_CHECK

const float EPSILON =.5;



void obstacle_collision(float *cx, float *cy, float *r, float *q_new, uint32_t *d_obs_flag);

void copyToDeviceMemory(void* d, void* h, size_t size);

void copyToHostMemory(void* h, void* d, size_t size);

void* allocateDeviceMemory(size_t size);

void freeDeviceMemory(void* d);


#endif
