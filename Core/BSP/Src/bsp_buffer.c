/*
 * bsp_buffer.c
 *
 *  Created on: 3 Mar 2022
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_buffer.h"

/*
 * Adds a byte to the end of the buffer
 * Add one byte at a time!
 */
void append_buffer(buffer_t* buffer, uint8_t data){
	buffer->last_time = HAL_GetTick();
	buffer->buffer[buffer->curr_byte] = data;
	buffer->stored_bytes++;
	buffer->curr_byte = (buffer->curr_byte >= BUFFER_SIZE-1) ? 0 : buffer->curr_byte + 1;
	if (buffer->stored_bytes >= BUFFER_SIZE) {
		buffer->stored_bytes = BUFFER_SIZE;
	}
}

void buffer_init(buffer_t *buffer)
{
	buffer->curr_byte = 0;
	buffer->last_proc = 0;
	buffer->stored_bytes = 0;
	buffer->last_time = 0;
}
