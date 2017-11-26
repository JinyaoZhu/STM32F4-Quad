#ifndef __COM_DRIVER_H__
#define __COMDRIVER_H__
#include "windows.h"

#define USED_COM    "COM3"

HANDLE Com_Open(void);

bool Com_Send(HANDLE fd, BYTE *data, DWORD dwExpectSend, DWORD* dwRealSend);

bool Com_Receive(HANDLE fd, BYTE *data, DWORD dwWantRead, DWORD* dwRealRead);

int Com_Close(void *fd);


#endif /*__COM_DRIVER_H__*/







