/* 
 * File:   storage.h
 * Author: C16107
 *
 * Created on May 30, 2023, 4:41 PM
 */

#ifndef STORAGE_H
#define	STORAGE_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include<string.h>
#include "system/fs/sys_fs.h"    

/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* Application's state machine's initial state. */
    /* The app waits for sdcard to be mounted */
    FS_MOUNT_WAIT = 0,

    /* Set the current drive */
    FS_SET_CURRENT_DRIVE,

    /* The app opens the file to read */
    FS_OPEN_FIRST_FILE,

        /* Create directory */
    FS_CREATE_DIRECTORY,

        /* The app opens the file to write */
    FS_OPEN_SECOND_FILE,

    /* The app reads from a file and writes to another file */
    FS_READ_WRITE_TO_FILE,

    /* The app closes the file*/
    FS_CLOSE_FILE,

    /* The app closes the file and idles */
    FS_IDLE,

    /* An app error has occurred */
    FS_ERROR

} FS_STATES;
    
    
    
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* SYS_FS File handle for 1st file */
    SYS_FS_HANDLE      fileHandle;

    /* SYS_FS File handle for 2nd file */
    SYS_FS_HANDLE      fileHandle1;

    /* Application's current state */
    FS_STATES         state;

    int32_t            nBytesRead;

    /* Flag to indicate SDCARD mount status */
    volatile bool      sdCardMountFlag;
} FS_DATA;    
    
    
    
    
    
typedef void (*STORAGE_READ_CALLBACK)(void);
typedef void (*STORAGE_WRITE_CALLBACK)(void);
void storageWriteInit(void);
void flash_write(uint32_t flash_addr,uint32_t data_size,uint8_t *data);
void closeFile(void);


#ifdef	__cplusplus
}
#endif

#endif	/* STORAGE_H */

