#include "com_driver.h"
#include "stdio.h"

HANDLE Com_Open(void)
{
	long error_status;
	//打开串口
	HANDLE m_hCom = CreateFile(TEXT(USED_COM), GENERIC_READ | GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (m_hCom == INVALID_HANDLE_VALUE)
	{
		error_status = GetLastError();
		printf("Create File fail:%d\n", error_status);
		return NULL;
	}
	else{
		printf("Open ");
		printf(USED_COM);
		printf(" succeed!\n");
	}

	//设置缓冲区大小
	if (!SetupComm(m_hCom, 1024, 1024))
	{
		printf("SetupComm fail!\n");
		CloseHandle(m_hCom);
		return NULL;
	}

	//设置超时
	COMMTIMEOUTS TimeOuts;

	memset(&TimeOuts, 0, sizeof(TimeOuts));

	TimeOuts.ReadIntervalTimeout = 100;
	TimeOuts.ReadTotalTimeoutConstant = 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = 100;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	TimeOuts.WriteTotalTimeoutMultiplier = 50;
	SetCommTimeouts(m_hCom, &TimeOuts);

	PurgeComm(m_hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);

	//设置串口参数
	DCB dcb = { 0 };

	if (!GetCommState(m_hCom, &dcb))
	{
		printf("GetCommState fail\n");
		return NULL;
	}

	dcb.DCBlength = sizeof(dcb);

	if (!BuildCommDCB(TEXT("115200,n,8,1"), &dcb)) //填充ＤＣＢ的数据传输率、奇偶校验类型、数据位、停止位
	{
		printf("BuildCommDCB fail\n");
		CloseHandle(m_hCom);
		return NULL;
	}

	if (SetCommState(m_hCom, &dcb))
	  return m_hCom;
	else
	  printf("SetCommState Error!\n");

	return NULL;
}


bool Com_Send(HANDLE fd, BYTE *data, DWORD dwExpectSend, DWORD* dwRealSend)
{
	DWORD dwError;

	BYTE *pSendBuffer;

	pSendBuffer = data;

	if (ClearCommError(fd, &dwError, NULL))
	{
		PurgeComm(fd, PURGE_TXABORT | PURGE_TXCLEAR);
	}

	if (WriteFile(fd, pSendBuffer, dwExpectSend, dwRealSend, NULL))
		return true;

	else{
		printf("TX error!\n");
		return false;
	}

}

bool Com_Receive(HANDLE fd, BYTE *data, DWORD dwWantRead, DWORD* dwRealRead)
{
	DWORD dwError;

	BYTE* pReadBuf;

	pReadBuf = data;

	if (ClearCommError(fd, &dwError, NULL))
	{
		PurgeComm(fd, PURGE_TXABORT | PURGE_TXCLEAR);
	}
	if (ReadFile(fd, pReadBuf, dwWantRead, dwRealRead, NULL))
		return true;
	else{
		printf("RX error!");
		return false;
	}
}


int Com_Close(void *fd)

{
	CloseHandle(fd);
	printf("Close ");
	printf(USED_COM);
	printf(". \n");
	return 0;
}