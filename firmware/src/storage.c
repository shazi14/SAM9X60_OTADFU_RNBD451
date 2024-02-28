/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    storage.c

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

#include "storage.h"
#include "definitions.h"  

uint8_t erase_cnt = 0;
bool flash_data_ready;
bool file_write_ready = false;
#define SDCARD_MOUNT_NAME    SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0
#define SDCARD_DEV_NAME      SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0
#define SDCARD_FILE_NAME     "harmony.bin"
#define SDCARD_DIR_NAME      "Dir1"
FS_DATA FSappData;
void input_task(void)
{
    
}
void flash_write(uint32_t flash_addr,uint32_t data_size,uint8_t *data)
{
 
    flash_data_ready = false;
}
void test_page_write(void)
{

    erase_cnt = 0;
}

int storageWrite(void* addr,uint8_t* data,uint32_t size,STORAGE_WRITE_CALLBACK cb)
{
    if(file_write_ready == false)
    {
        if((FSappData.sdCardMountFlag == true) && (FSappData.state != FS_ERROR))
        {
             if(SYS_FS_CurrentDriveSet(SDCARD_MOUNT_NAME) == SYS_FS_RES_FAILURE)
             {
                 FSappData.state = FS_ERROR;
             }
             else
             {
                SYS_FS_FileDirectoryRemove(/*SDCARD_DIR_NAME"/" */SDCARD_FILE_NAME);
          
                FSappData.fileHandle1 = SYS_FS_FileOpen(/*SDCARD_DIR_NAME"/"*/SDCARD_FILE_NAME,
                (SYS_FS_FILE_OPEN_WRITE));

                if(FSappData.fileHandle1 == SYS_FS_HANDLE_INVALID)
                {
                    /* Could not open the file. Error out*/
                    FSappData.state = FS_ERROR;
                }
                else
                {
                    /* If read was success, try writing to the new file */
                    if(SYS_FS_FileWrite(FSappData.fileHandle1, (const void *)data, size) == -1)
                    {
                        /* Write was not successful. Close the file
                         * and error out.*/
                        SYS_FS_FileClose(FSappData.fileHandle1);
                        FSappData.state = FS_ERROR;
                    }
                    else
                    {
                        file_write_ready = true;
                    }
                }
             }
        }
        else
        {
            //SD Card not mounted
        }
    }
    else // writing 2nd frame of data onward
    {
        if(SYS_FS_FileWrite(FSappData.fileHandle1, (const void *)data, size) == -1)
        {
            /* Write was not successful. Close the file
             * and error out.*/
            SYS_FS_FileClose(FSappData.fileHandle1);
            FSappData.state = APP_ERROR;
            file_write_ready = false;
        }
    }
    if(cb != NULL)
    {
        cb();
    }
    return 0;
}
int storageRead(void* addr,uint8_t* data,uint32_t size, STORAGE_READ_CALLBACK cb)
{

    return 1;
}
int storageWrite2Page(void* addr,uint8_t* data,uint32_t size,STORAGE_WRITE_CALLBACK cb)
{

    return 0;
}


static void APP_SysFSEventHandler(SYS_FS_EVENT event,void* eventData,uintptr_t context)
{
    switch(event)
    {
        /* If the event is mount then check if SDCARD media has been mounted */
        case SYS_FS_EVENT_MOUNT:
            if(strcmp((const char *)eventData, SDCARD_MOUNT_NAME) == 0)
            {
                FSappData.sdCardMountFlag = true;
            }
            break;

        /* If the event is unmount then check if SDCARD media has been unmount */
        case SYS_FS_EVENT_UNMOUNT:
            if(strcmp((const char *)eventData, SDCARD_MOUNT_NAME) == 0)
            {
                FSappData.sdCardMountFlag = false;

                FSappData.state = FS_MOUNT_WAIT;
                
            }

            break;

        case SYS_FS_EVENT_ERROR:
        default:
            break;
    }
}
void closeFile(void)
{
    SYS_FS_FileClose(FSappData.fileHandle1);
    FSappData.state = FS_IDLE;
}

void storageWriteInit(void)
{
    if(FSappData.sdCardMountFlag == true)
    {
        FSappData.state = FS_SET_CURRENT_DRIVE;
        
    }
    else
    {
        FSappData.state = FS_MOUNT_WAIT;
        file_write_ready = false;
        SYS_FS_EventHandlerSet((void const*)APP_SysFSEventHandler,(uintptr_t)NULL);
    }
}
