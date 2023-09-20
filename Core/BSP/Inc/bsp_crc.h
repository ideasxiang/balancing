/*
 * bsp_crc.h
 *
 *  Created on: 8 Dec 2021
 *      Author: wx
 */

#ifndef BSP_INC_BSP_CRC_H_
#define BSP_INC_BSP_CRC_H_

unsigned char get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);


#endif /* BSP_INC_BSP_CRC_H_ */
