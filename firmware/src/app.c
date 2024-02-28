/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

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
#include <string.h>
#include "app.h"
#include "configuration.h"
#include "system/debug/sys_debug.h"
#include "user.h"
#include "uart.h"
#include "bootloader.h"
#include "storage.h"
#include "definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#ifdef FS_Code
#define SDCARD_MOUNT_NAME    SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0
#define SDCARD_DEV_NAME      SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0
#define SDCARD_FILE_NAME     "FILE_TOO_LONG_NAME_EXAMPLE_123.JPG"
#define SDCARD_DIR_NAME      "Dir1"
#endif
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
bool ack_sent = false;
uint32_t cnt,rcvdfrm;
volatile uint8_t rx_data_rcvd;
extern UART_FIFO UartRxBuffer;
uint8_t cmd[] = {'$','$','$'};
uint8_t sendData[25];
extern bool reboot_done;
uint32_t prev_ptr;
uint8_t OTAA1_ACK[] = {'O','T','A','A',',','0','1',',','0','2','1','3','\r','\n'};
uint8_t OTAA2_ACK[] = {'O','T','A','A',',','0','2','\r','\n'};
uint8_t OTAAV01_ACK[] = {'O','T','A','V',',','0','1','\r','\n'};
uint8_t OTAAV00_ACK[] = {'O','T','A','V',',','0','0','\r','\n'};
int payload_hdrlen;
volatile bool send_OTAA_Ack = false;
extern bool ota_done;

extern bool ota_req,ota_complete,send_otav00;
extern uint32_t ImageLength;
extern bool break_here;
/* Application data buffer */
#ifdef FS_Code
static uint8_t BUFFER_ATTRIBUTES dataBuffer[APP_DATA_LEN];
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
#ifdef FS_Code
static void APP_SysFSEventHandler(SYS_FS_EVENT event,void* eventData,uintptr_t context)
{
    switch(event)
    {
        /* If the event is mount then check if SDCARD media has been mounted */
        case SYS_FS_EVENT_MOUNT:
            if(strcmp((const char *)eventData, SDCARD_MOUNT_NAME) == 0)
            {
                appData.sdCardMountFlag = true;
            }
            break;

        /* If the event is unmount then check if SDCARD media has been unmount */
        case SYS_FS_EVENT_UNMOUNT:
            if(strcmp((const char *)eventData, SDCARD_MOUNT_NAME) == 0)
            {
                appData.sdCardMountFlag = false;

                appData.state = APP_MOUNT_WAIT;

                LED_OFF();
            }

            break;

        case SYS_FS_EVENT_ERROR:
        default:
            break;
    }
}
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_RNBD_INIT;
    ack_sent = 0;
    rcvdfrm = 0;
    rx_data_rcvd = 0;
    FLEXCOM7_USART_Initialize();
    
#ifdef FS_Code
    /* Register the File System Event handler */
    SYS_FS_EventHandlerSet((void const*)APP_SysFSEventHandler,(uintptr_t)NULL);
#endif
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {

        case APP_ERROR:
            /* The application comes here when the demo has failed. */
            break;
        case APP_RNBD_INIT:
            FLEXCOM7_USART_Read(UartRxBuffer.data,sizeof(UART_BUFFER_LEN));
            cnt = 0;
            appData.state = APP_RNBD_START;
            storageWriteInit();
            break;
        case APP_RNBD_START:
            if(FLEXCOM7_USART_ReadCountGet() > cnt)
            {
                cnt += FLEXCOM7_USART_ReadCountGet();
                sendData[0] = '+';
                sendData[1] = '\r';
                sendData[2] = '\n';
                FLEXCOM7_USART_Write(sendData,3);
            }
            break;
            case APP_RNBD_EVENT_RCVD:
            {
                process_rx_data();
                prev_ptr = UartRxBuffer.WritePtr;
                rcvdfrm++;
                appData.state = APP_RNBD_IDLE;
            }
            break;
            case APP_RNBD_IDLE:
            if(ota_req == true)
            {
                LED_TOGGLE();
                if(ack_sent == 0)
                {
                    FLEXCOM7_USART_Write(OTAA1_ACK, sizeof(OTAA1_ACK));
                    ack_sent = true;
                }
                appData.state = APP_RNBD_DATA_REQ;
                imageBegin(4,(int *)&ImageLength);
               
            }
            else if (reboot_done == true)
            {
                FLEXCOM7_USART_Write(cmd,sizeof(cmd));
                reboot_done = false;
            }
            if(rx_data_rcvd > 0)
            {
                process_rx_data();
                rx_data_rcvd--;
            }
            break;
        case APP_RNBD_DATA_REQ:
            break;
        case APP_RNBD_DATA_PROC:
            {
                payload_hdrlen = copy_rx_data();
                if(send_OTAA_Ack == true)
                {
                    FLEXCOM7_USART_Write(OTAA2_ACK, sizeof(OTAA2_ACK));
                    send_OTAA_Ack = false;
                }
                appData.state = APP_RNBD_DATA_REQ;
            }
            break;
        case APP_RNBD_DATA_END:
            if(send_OTAA_Ack == true)
            {
                send_OTAA_Ack = false;
                FLEXCOM7_USART_Write(OTAAV01_ACK, sizeof(OTAAV01_ACK));
            }
            appData.state = APP_RNBD_SEND_OTAV00;
            send_OTAA_Ack = true;
            break;
        case APP_RNBD_SEND_OTAV00:
            if(send_otav00 == true)
            {
                send_OTAA_Ack = false;
                FLEXCOM7_USART_Write(OTAAV00_ACK, sizeof(OTAAV00_ACK));
                appData.state =APP_RNBD_DATA_REQ;
            }
            else if (rx_data_rcvd > 0)
            {
                process_rx_data();
                rx_data_rcvd--;
            }
            break;
        case APP_RNBD_WAIT_OTA_COMPLETE:
            if(ota_complete == true)
            {  
//                FLEXCOM7_USART_Write(OTAA2_ACK, sizeof(OTAA2_ACK));
                FLEXCOM7_USART_Write(OTAAV01_ACK, sizeof(OTAAV01_ACK));
                FLEXCOM7_USART_Write(OTAAV00_ACK, sizeof(OTAAV00_ACK));
                appData.state = APP_RNBD_SEND_OTAV00;
//                send_otav00 = true;
                rx_data_rcvd = 1;
//                ota_done = false;
            }
            else if(rx_data_rcvd >0)
            {
                process_rx_data();
                rx_data_rcvd--;
            }
            
            break;

        default:
            break;
    }
}

void chnage_ota_state(void)
{
    appData.state = APP_RNBD_DATA_PROC;
    send_OTAA_Ack = true;
}
void set_OTA_end_flag()
{
    appData.state = APP_RNBD_WAIT_OTA_COMPLETE;
    break_here = true;
    rx_data_rcvd=1;
}
void chnage_app_state(void )
{
    appData.state = APP_RNBD_EVENT_RCVD;
    rx_data_rcvd= rx_data_rcvd + 1;
    LED_TOGGLE();
}
void send_OTAV_ack(void)
{
    rx_data_rcvd=1;
    send_OTAA_Ack = true;
}
/*******************************************************************************
 End of File
 */
