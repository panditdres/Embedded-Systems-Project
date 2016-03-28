#include "filter.h"
#include <stdio.h>
/*
	Initializes the Moving Average Filter of specified size.
	Input:
	ring_buffer_t *buffer = pointer to Buffer definition
	int *arr = pointer to buffer array 
	int buffer_size = desired buffer size
*/
void init_buffer(ring_buffer_t *buffer, int *arr, int buffer_size) {
	int i;
	for (i = 0; i < buffer_size; i++) 
		arr[i] = 0; 
	
	buffer->array = arr;
	buffer->size = buffer_size;
	buffer->current = 0;	
	buffer->denominator = ((float) buffer_size * ((float) buffer_size + 1)) / 2;	
}
/*
	Filter Data Point.
	Input:
	int16_t data_point = data point to be filtered 
	ring_buffer_t* buffer = pointer to moving average filter
	Return;
	(Float) Average of the Filter

*/
float filter_point(int16_t data_point, ring_buffer_t* buffer) {
	buffer->array[buffer->current] = data_point;
	int size = buffer->size;		
	int i = (buffer->current + 1) % size;
	if (buffer->current == size)
		buffer->current = 0;
	int j;
	int numerator = 0;
	
	for (j = 0; j < size; j++) {
		numerator += (j + 1) * buffer->array[i];		
		i += 1;
		if (i == size)
			i = 0;
	}
	
	buffer->current += 1;
	if (buffer->current == size)
		buffer->current = 0;

	return ((float) numerator) / buffer->denominator;
}