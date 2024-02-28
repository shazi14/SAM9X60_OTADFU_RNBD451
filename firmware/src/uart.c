/******************************************/
#include"uart.h"
#include"app.h"
#include"bootloader.h"
#include <string.h>

UART_FIFO UartRxBuffer;
volatile uint32_t counter;
bool ble_connected,update_parm,file_open,ota_req,dfu_mode;
uint8_t connection_hndl[4];
uint8_t mac_id[12];
uint32_t ImageLength;
uint32_t image_id;
uint8_t image_version[8];
uint16_t rcvd_crc16,rcvd_checksum;
bool reboot_done, ota_start,ota_done, ota_complete,send_otav00;
int payloadfrm= 0;
bool break_here;
uint8_t image_data[MAX_OTA_PKT_LEN + 200];
int short_len;
uint16_t payload_len;
uint32_t rcvddata, payloadlen;
void uart_init(void)
{
    
    UartRxBuffer.ReadPtr = 0;
    UartRxBuffer.WritePtr = 0;
    update_parm = false;
    file_open = false;
    reboot_done = false;
    ota_start = false;
    counter = 0;
    ble_connected = false;
    dfu_mode = false;
    payloadlen = 0;
    rcvddata = 0;
    ota_req = false;
    ota_done = false;
    ota_complete = false;
    send_otav00 = false;
    ImageLength = 0;
    memset(UartRxBuffer.data,0,UART_BUFFER_LEN);
    
}
void uart_callback(uint8_t byte_rcvd)
{
    UartRxBuffer.data[UartRxBuffer.WritePtr++] = byte_rcvd;
    if(UartRxBuffer.WritePtr >=UART_BUFFER_LEN)
    {
        UartRxBuffer.WritePtr = 0;
    }
    if(ota_start == true)
    {
        if(ota_done == true)
        {
            if(byte_rcvd == '%')
            {
                counter++;
                if((counter & 1) == 0)
                {
                    if(ota_complete == false)
                    {
                        set_OTA_end_flag();
                    }
                    else
                    {
                        send_OTAV_ack();
                        ota_start = false;
                        ota_req = false;
                    }
                }
            }
        }
        else
        {
            UartRxBuffer.byteCnt++;
            if(UartRxBuffer.byteCnt >= (UartRxBuffer.PktLen )) //Flexcom 3 byte
            {
                UartRxBuffer.byteCnt = 0;
                chnage_ota_state();
            }
        }
    }
    else if(byte_rcvd == '%')
    {
        counter++;
        if((counter & 1) == 0)
        {
            if(ota_done ==false)
            {
                chnage_app_state();
            }
            else
            {
                send_OTAV_ack();
            }
        }
            
    }
}
void process_rx_data(void)
{
    int i;
    uint32_t ret_val;
    int len = UartRxBuffer.WritePtr - UartRxBuffer.ReadPtr;
    uint8_t *data = &UartRxBuffer.data[UartRxBuffer.ReadPtr];
    if(reboot_done == false)   
    {
        for(i=0;i<len;i++)
        {
             if(data[i] == '%' && data[i+1] == 'R' && data[i+2] == 'E' && data[i+3] == 'B' && data[i+5]=='O' && data[i+6]=='T')
             {
                 UartRxBuffer.ReadPtr= UartRxBuffer.ReadPtr+(i+8);
                 reboot_done = true;
                 break;
             }
        }
    }
    if(ble_connected == false)
    {
        for(i=0;i<len;i++)
        {
            if(data[i] == '%' && data[i+1] == 'C' && data[i+2] == 'O' && data[i+3] == 'N' && data[i+6]=='C' && data[i+7]=='T')
            {
                ble_connected = true;
                update_parm = true;
                file_open = false;
                UartRxBuffer.ReadPtr= UartRxBuffer.ReadPtr + (i+8);
                break;
            }
        }
    }
    else //if(ble_connected == true)
    {
        for(i=0;i<len;i++)
        {
            if(data[i]=='%' && data[i+1] == 'S' && data[i+2] == 'E' && data[i+3] == 'U' && data[i+7]=='D')
            {
                dfu_mode = true;
                UartRxBuffer.ReadPtr = UartRxBuffer.ReadPtr + i+9;
                break;
            }
        }

        for(i=0;i<len;i++)
        {
            if(data[i] == '%' && data[i+1] == 'O' && data[i+2] == 'T' && data[i+3] == 'A' && data[i+4]=='_' && data[i+6]=='E' && data[i+7]=='Q')
            {
                ota_req = true;
                ret_val = extract_ota_parm(&data[i+9]); 
                UartRxBuffer.ReadPtr = UartRxBuffer.ReadPtr+(ret_val + i+9);
                break;
            }
        }
        if(ota_req == true)
        {
            for(i=0;i<len;i++)
            {
                if(data[i] == '%' && data[i+1] == 'O' && data[i+2] == 'T' && data[i+3] == 'A' && data[i+4]=='_' && data[i+5]=='S' && data[i+9]=='T')
                {

                    UartRxBuffer.ReadPtr +=(i+10+4);
                    ota_start = true;
                    UartRxBuffer.byteCnt = 0;
                    UartRxBuffer.PktLen = MAX_OTA_PKT_LEN;
                    break;
                }
            }
        }
        if(ota_complete == false)
        {
            for(i=0;i<len;i++)
            {
                if(data[i] == '%' && data[i+1] == 'O' && data[i+2] == 'T' && data[i+3] == 'A' && data[i+4]=='_' && data[i+7]=='M' && data[i+11]=='T' && data[i+12]=='E')
                {
                    ota_complete = true;
                    UartRxBuffer.ReadPtr = UartRxBuffer.ReadPtr+ i+13;
                    break;
                }
            }
        }
        if(ota_complete == true)
        {
            for(i=0;i<len;i++)
            {
                if(data[i] == 'A' && data[i+1] == 'O' && data[i+2] == 'K' && data[i+5]=='D' && data[i+6]=='F' && data[i+7]=='U' && data[i+8] == '>')
                {
                    send_otav00 = true;
                    UartRxBuffer.ReadPtr = UartRxBuffer.ReadPtr+ i+10;
                    break;
                }
            }
        }
    }
}
uint8_t ascii2num(uint8_t num)
{
    uint8_t val=0;
    if(num >='0' && num <= '9')
    {
        val = num -'0';
    }
    else if(num >='A' && num <='F')
    {
        val = 10+ num - 'A';
    }
    else if(num >='a' && num <='f')  
    {
        val = 10+ num - 'a';
    }
    return val;
}
int extract_ota_parm(uint8_t img_data[])
{
    int i,idx;
    for(i=0;i<4;i++)
    {
        connection_hndl[i] = img_data[i];
    }
    idx = 4+1; //connection handle + ,
    ImageLength = 0;
    for(i=idx;i<(idx+8);i++)
    {
         ImageLength <<=4;
         ImageLength |= ascii2num(img_data[i]);
    }
    idx = idx+1+8; // length
    image_id = 0;
    for(i=idx;i <(idx+8);i++)
    {
        image_id <<=4;
        image_id |= ascii2num(img_data[i]);
    }
    idx = idx+1+8; //image_id
    for(i=0;i<8;i++)
    {
        image_version[i] = img_data[i+idx];
    }
    idx = idx+1+8; //image_version
    rcvd_crc16 = 0;
    for(i=idx;i<(idx +4);i++)
    {
        rcvd_crc16 <<=4;
        rcvd_crc16 |= ascii2num(img_data[i]);
    }
    idx = idx+1+4; //CRC16
    rcvd_checksum = 0;
    for(i=idx;i<(idx +4);i++)
    {
        rcvd_checksum <<=4;
        rcvd_checksum |= ascii2num(img_data[i]);
    }
    idx = idx+1+4; //Checksum
    return idx;
}
int find_marker(uint8_t img_data[],int len)
{
    int ret_val = 0, i;
    for(i=0;i<len;i++)
    {
        if(img_data[i] == '%')
        {
            ret_val = i;
            break;
        }
    }
    return (ret_val);
}

int copy_rx_data(void)
{
    int i,idx;
    int ret_val = 0;
    if(payloadfrm >= 1)
    {
        break_here = true;
    }
    copy_image();
    for(i=0;i<MAX_OTA_PKT_LEN;i++)
    {
        if(image_data[i] == '%' && image_data[i+1] == 'O' && image_data[i+2] == 'T' && image_data[i+3] == 'A' && image_data[i+4]=='_' && image_data[i+5]=='D' && image_data[i+8]=='A')
        {
            short_len = 0;
            payload_len = convert_to_decimal(&image_data[i+13], 4);
            rcvddata +=payload_len;
            payloadlen+=payload_len;
 
            ret_val = i+13+4+1;
            payloadfrm++;
            idx = MAX_OTA_PKT_LEN + 10;

            if(ImageLength <= (rcvddata + MAX_OTA_PKT_LEN - 19))
            {
                idx = ImageLength - rcvddata + 10 + 19;
            }
            
            storeImageChunk(payload_len,  &image_data[ret_val]); // 
             __disable_irq();
             UartRxBuffer.PktLen = idx;
             __enable_irq();
             
            
            if(rcvddata >= ImageLength)
            {
                break_here = true;
                ota_done = true;
              //  OTA_eof();
            }
            
            break;
        }
    }
    
    return ret_val;
}


void copy_image(void)
{
    int start = UartRxBuffer.ReadPtr;
    int end = UartRxBuffer.WritePtr,i;
    int len,len1;
    //extract image data
    if(start < end) // read ptr is behind write ptr. no brainer, copy the image
    {
        for(i=0;i<UartRxBuffer.PktLen;i++)
        {
            image_data[i] = UartRxBuffer.data[start+i];
        }
        UartRxBuffer.ReadPtr += UartRxBuffer.PktLen;
    }
    else if((start + UartRxBuffer.PktLen) <= UART_BUFFER_LEN)
    {
        for(i=0;i<UartRxBuffer.PktLen;i++)
        {
            image_data[i] = UartRxBuffer.data[start+i];
        }
        UartRxBuffer.ReadPtr += UartRxBuffer.PktLen;
        if(UartRxBuffer.ReadPtr == UART_BUFFER_LEN)
        {
            UartRxBuffer.ReadPtr = 0;
        }
    }
    else // Read Ptr is ahead of Write Ptr, Write Ptr is rolled back, special care needs to be taken
    {
        len = UART_BUFFER_LEN - UartRxBuffer.ReadPtr;
        for(i=0;i<len;i++)
        {
            image_data[i] = UartRxBuffer.data[start+i];
        }
        len1 = UartRxBuffer.PktLen - len;
        for(i=0;i<len1;i++)
        {
            image_data[len+i] = UartRxBuffer.data[i];
        }
        UartRxBuffer.ReadPtr = len1;
    }
    
}
int convert_to_decimal(uint8_t img_data[], uint8_t len)
{
    int ret_val = 0,i;
    for(i=0;i<len;i++)
    {
        if(img_data[i] == ',')
        {
            break;
        }
        ret_val<<=4;
        ret_val |= ascii2num(img_data[i]);
    }
    return ret_val;
}
void adjust_pkt_len(uint32_t length)
{
    UartRxBuffer.PktLen = length;
}