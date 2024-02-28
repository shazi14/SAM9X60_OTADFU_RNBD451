/* 
 * File:   bootloader.h
 * Author: C16107
 *
 * Created on May 23, 2023, 2:20 PM
 */

#ifndef BOOTLOADER_H
#define	BOOTLOADER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "bsp/bsp.h"
typedef void (*STORAGE_READ_CALLBACK)(void);
typedef void (*STORAGE_WRITE_CALLBACK)(void);
int storageRead(void* addr,uint8_t* data,uint32_t size, STORAGE_READ_CALLBACK cb);
int storageWrite(void* addr,uint8_t* data,uint32_t size,STORAGE_WRITE_CALLBACK cb);
struct boot_image_header_t
{
    uint32_t signature;
    uint32_t size;
};
void imageBegin(uint32_t size, int* data);
uint16_t ota_crc_calc_finish(void);
void ota_crc_calc_update(int count, uint8_t ptr[]);
void storeImageChunk(uint32_t size, uint8_t* data);
void adjust_pkt_len(uint32_t);
void set_dfu_end_state(void);
void write_signature(void);
int storageWrite2Page(void* addr,uint8_t* data,uint32_t size,STORAGE_WRITE_CALLBACK cb);
int check_for_new_image(void);
int run_newApplication(uint32_t address);
#ifdef	__cplusplus
}
#endif

#endif	/* BOOTLOADER_H */

