/*
 * bsp_buffer.h
 *
 *  Created on: 3 Mar 2022
 *      Author: wx
 */

#ifndef BSP_INC_BSP_BUFFER_H_
#define BSP_INC_BSP_BUFFER_H_



#define BUFFER_SIZE 24
typedef struct {
	uint8_t buffer[BUFFER_SIZE];
	uint8_t curr_byte;
	uint8_t stored_bytes;
	uint32_t last_time;
	uint8_t last_proc;
}buffer_t;
void append_buffer(buffer_t* buffer, uint8_t data);
void buffer_init(buffer_t* buffer);

#endif /* BSP_INC_BSP_BUFFER_H_ */
