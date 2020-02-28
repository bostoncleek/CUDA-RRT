#ifndef COLLISON_CHECK
#define COLLISON_CHECK

const float EPSILON =.5;


<<<<<<< HEAD
void obstacle_collision(float *cx, float *cy, float *r, float *q_new, uint32_t *d_obs_flag);

=======
>>>>>>> 61ee965c0b7fe43c9060448bfa43e9b086ec1d0b

void obstacle_collision(float *cx, float *cy, float *r, float *q_new, uint32_t *d_obs_flag);

void copyToDeviceMemory(void* d, void* h, size_t size);

void copyToHostMemory(void* h, void* d, size_t size);

void* allocateDeviceMemory(size_t size);

void freeDeviceMemory(void* d);


#endif
