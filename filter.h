#ifndef __filter
#define __filter
#endif

#include "stdint.h"


typedef struct
{
	int *array;
	int size;
	volatile uint16_t current; 	
	float denominator;
} ring_buffer_t;

void init_buffer(ring_buffer_t *buffer, int *arr, int buffer_size);
float filter_point(int16_t new_data_point, ring_buffer_t* buffer);
