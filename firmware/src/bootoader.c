/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    bootloader.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "bootloader.h"
#include "user.h"
#include "storage.h"
#include "definitions.h"  
#include <string.h>
#define BOOT_CHUNK_SIZE 4096
#define BOOT_SIG_SIZE 0x200
#define BOOT_IMAGE_ADDRESS 0x10000
#define BOOT_SIG_ADDRESS  (BOOT_IMAGE_ADDRESS - BOOT_SIG_SIZE)
#define BOOT_IMAGE_SIGNATURE 0xB00710AD
#define IMAGE_END_SIZE 9
#define APP_START_ADDRESS                       (0x2000UL)
#define BTL_TRIGGER_RAM_START                   0x20000000
#define BTL_TRIGGER_PATTERN (0x5048434DUL)
static uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

static uint32_t total_expected;
uint32_t total_received;
static uint32_t chunk_address;
uint32_t boot_chunk_len;
//static uint32_t calc_chksum = 0;
uint8_t BUFFER_ATTRIBUTES boot_chunk[BOOT_CHUNK_SIZE];

static int led = 0;
uint8_t flash_write_done;
uint8_t chk_start;
uint32_t frame_start,frmcnt;
extern bool break_here;
uint32_t send_ack;
uint8_t header_info[50];
uint32_t header_len;
void imageBegin(uint32_t size, int* data)
{
    if(size == sizeof(int))
    {
        total_expected = *data;
        total_received = 0;

        boot_chunk_len = 0;

        chunk_address = 0;
        LED1_OFF();
        LED2_ON();
        storageWriteInit();
        frame_start = 0;
        frmcnt = 0;
        send_ack = 0;
    }
}
#if 1
static void read_cb(void)
{ 
}
#endif
static void write_cb(void)
{
    flash_write_done = 1;
    send_ack++;
}
void shiftLED(void)
{
    led = led^1;
     LED2_OFF();
     LED1_OFF();
     if(led == 0)
     {
         LED2_ON();
     }
     else
     {
         LED1_ON();
     }
}
static void imageEnd(void)
{
    closeFile();
}
void storeImageChunk(uint32_t size, uint8_t* data)
{
    unsigned int* ptr;
    int sz;//,leftover;
    
    if(size < BOOT_CHUNK_SIZE - boot_chunk_len)
    {
        memcpy(&boot_chunk[boot_chunk_len], data, size);
        boot_chunk_len += size;
    }
    /* boot chunk buffer is full, send to SPI flash */
    else
    {
        sz = BOOT_CHUNK_SIZE - boot_chunk_len;
        
        memcpy(&boot_chunk[boot_chunk_len], data, sz);
        flash_write_done = 0;
        
        ptr = (unsigned int*)(BOOT_IMAGE_ADDRESS  + chunk_address);
 
        storageWrite(ptr, boot_chunk, BOOT_CHUNK_SIZE, &write_cb);
                
        shiftLED();
      //  flash_write_done = 1; // testing purpose
        frame_start++;
        
        /* accomodate dangling data */
        chunk_address += BOOT_CHUNK_SIZE;
        boot_chunk_len = size - (BOOT_CHUNK_SIZE - boot_chunk_len);
        memcpy(boot_chunk, data + sz, boot_chunk_len);

    }
    if(chk_start)
    {
        if((total_received + size) < (total_expected -IMAGE_END_SIZE))
        {
           // ota_crc_calc_update(size, (uint8_t*)data);
        }
        else
        {
            
           // leftover = total_expected - total_received -9;
            //ota_crc_calc_update(leftover, (uint8_t*)data);
            //calc_chksum = ota_crc_calc_finish();
//            if(checksum == calc_chksum)
//            {
//                data_integrity = 1;
//            }
//            else
//            {
//                data_integrity = -1;
//            }
            chk_start = 0;
        }
    }
    frmcnt++;
    total_received += size;
    if((total_received + size) >= total_expected)
    {
        adjust_pkt_len(total_expected-total_received);
    }
    if(frame_start >= 2)
    {
        break_here = true;
    }
    if(frmcnt >=101)
    {
         break_here = true;
    }
    /* send the last partial chunk to SPI flash and finish up */
    if(total_received >= total_expected)
    {

        storageWrite((void*)(BOOT_IMAGE_ADDRESS + chunk_address),
                     boot_chunk,
                     boot_chunk_len,
                     &write_cb);

        shiftLED();
        write_signature();
        set_dfu_end_state();
        imageEnd();
        
    }
}
void set_dfu_end_state()
{
    
}
static uint16_t CRC_Value = 0xFFFF;
uint16_t SLIP_FINISH_CRC(uint16_t crc)
{
    int crc_val;
    crc_val = (uint16_t)((crc & 0xFF00) >> 8) | ((crc & 0x00FF) << 8);
    crc_val = (uint16_t)((crc_val & 0xF0F0) >> 4) | ((crc_val & 0x0F0F) << 4);
    crc_val = (uint16_t)((crc_val & 0xCCCC) >> 2) | ((crc_val & 0x3333) << 2);
    crc_val = (uint16_t)((crc_val & 0xAAAA) >> 1) | ((crc_val & 0x5555) << 1);
    return (uint16_t)crc_val;
}
uint16_t crc_table[]= 
{
    0x0000, 0x1081, 0x2102, 0x3183,
    0x4204, 0x5285, 0x6306, 0x7387,
    0x8408, 0x9489, 0xa50a, 0xb58b,
    0xc60c, 0xd68d, 0xe70e, 0xf78f
};

uint16_t SLIP_UPDATE_CRC(uint16_t crc, uint16_t value)
{
    int crc_val;
    crc_val = (uint16_t)(crc >> 4) ^ crc_table[(crc ^ value) & 0x0F];
    crc_val = (crc_val >> 4) ^ crc_table[(crc_val ^ (value >> 4)) & 0x0F];
    return (uint16_t)crc_val;
}

void ota_crc_calc_update(int count, uint8_t ptr[])
{
    int i;
    for (i = 0; i < count; i++)
    {
        CRC_Value = SLIP_UPDATE_CRC(CRC_Value, ptr[i]);
    }
}

uint16_t ota_crc_calc_finish()
{
    uint16_t crc;
    CRC_Value = SLIP_FINISH_CRC(CRC_Value);
    crc = CRC_Value;
    CRC_Value = 0xFFFF;
    return crc;
}
int storageRead1(void* addr,uint8_t* data,uint32_t size, STORAGE_READ_CALLBACK cb)
{
    return 1;
}
int storageWrite1(void* addr,uint8_t* data,uint32_t size,STORAGE_WRITE_CALLBACK cb)
{
    
    return 1;
}

void write_signature()
{
    struct boot_image_header_t hdr;
    int len = sizeof(struct boot_image_header_t);
    storageWriteInit();
    /* boot image is good, write signature header to indicate validity */
    hdr.signature = BOOT_IMAGE_SIGNATURE;
    hdr.size = 4;
    memset(boot_chunk, 0, BOOT_CHUNK_SIZE);
    memcpy(boot_chunk, &hdr, sizeof(struct boot_image_header_t));
    memcpy(&boot_chunk[len], header_info, header_len);
    /* write signature */
    storageWrite2Page((void*)(BOOT_SIG_ADDRESS),boot_chunk, BOOT_CHUNK_SIZE,&write_cb);
    memset(boot_chunk, 0, BOOT_CHUNK_SIZE);
    storageRead((void*)(BOOT_SIG_ADDRESS),boot_chunk,BOOT_CHUNK_SIZE,&read_cb);
    frame_start = (uint32_t) boot_chunk[0];
  //  NVMCTRL_RegionLock((uint32_t)(BOOT_SIG_ADDRESS));
}
/*=============*/
int check_for_new_image()
{
//    uint32_t val;
    int ret_val = 0;
    /* Check for Bootloader Trigger Pattern in first 16 Bytes of RAM to enter
     * Bootloader.
     */
    if (BTL_TRIGGER_PATTERN == ramStart[0] && BTL_TRIGGER_PATTERN == ramStart[1] &&
        BTL_TRIGGER_PATTERN == ramStart[2] && BTL_TRIGGER_PATTERN == ramStart[3])
    {
        ramStart[0] = 0;
        ret_val = 1;
    }
    return ret_val;
}

void __WEAK SYS_DeInitialize( void *data )
{
    /* Function can be overriden with custom implementation */
}

int run_newApplication(uint32_t address)
{
    uint32_t msp            = *(uint32_t *)(address);
//    uint32_t reset_vector   = *(uint32_t *)(address + 4);
   
    if(msp == 0xffffffff)
    {
        return 0;
    }
    SYS_DeInitialize(NULL);
//    __set_MSP(msp);
 //   asm("bx %0"::"r"(reset_vector));
    return 1;
}