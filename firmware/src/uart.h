/* 
 * File:   uart.h
 * Author: Shazi Hashmi
 *
 * Created on October 12, 2023, 4:18 PM
 */
#include <stdint.h>
#include <stdbool.h>
#define UART_BUFFER_LEN 1024
#define MAX_OTA_PKT_LEN 0x213
#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t ReadPtr;
    uint32_t WritePtr;
    uint32_t byteCnt;
    uint32_t PktLen;
    uint8_t data[UART_BUFFER_LEN];
}UART_FIFO;

void uart_init(void);
void uart_callback(uint8_t);
int extract_ota_parm(uint8_t img_data[]);
uint8_t ascii2num(uint8_t num);
int find_marker(uint8_t img_data[],int len);
void chnage_ota_state(void);
int convert_to_decimal(uint8_t img_data[], uint8_t len);
void copy_image(void);
int copy_rx_data(void);
void set_OTA_end_flag(void);
void OTA_eof(void);
#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

