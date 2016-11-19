/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "WinCEBaroPort.hpp"
#include <stdio.h>
/*#include "Asset.hpp"
#include "OS/LogError.hpp" */
#include "OS/Sleep.h"

#ifdef _WIN32_WCE
#include "Widcomm.hpp"
#else
#include "OS/OverlappedEvent.hpp"
#endif

#include <windows.h>

#include <algorithm>
#include <assert.h>
#include <tchar.h>
#include <stdio.h>



WinCEBaroPort::WinCEBaroPort(PortListener *_listener, DataHandler &_handler)
  :Port(_listener, _handler), StoppableThread("SerialPort"), pDeviceName(nullptr)
{
}

WinCEBaroPort::~WinCEBaroPort()
{
  // Close the communication port.
  if (GetState() == PortState::READY) {
    StoppableThread::BeginStop();
    Thread::Join();
  }
}


WinCE_DeviceType WinCEBaroPort::GetDeviceType() {
	static int rc = -1;
	if(rc == -1) {
		HKEY hKey;
		if( ERROR_SUCCESS==RegOpenKeyEx( HKEY_LOCAL_MACHINE, L"Ident", 0,KEY_READ,&hKey)){
			wchar_t	buffer[512];
			DWORD	size=sizeof(buffer);
			if(ERROR_SUCCESS == RegQueryValueEx(hKey,L"Name",nullptr,nullptr,(LPBYTE) buffer,&size)){
				if(wcscmp(L"S3867", buffer) == 0) {
					rc = S3867;
				} else if(wcscmp(L"S3877", buffer) == 0) {
					rc = S3877;
				}
			}
		}
    if(rc == -1 && ERROR_SUCCESS==RegOpenKeyEx( HKEY_LOCAL_MACHINE,
                      L"Drivers\\BuiltIn\\YFPressure", 0,KEY_READ,&hKey)){
      rc = BikePilot;
    }
		if(rc == -1)
			rc=UNKNON;
	}
	return (WinCE_DeviceType) rc;
}

bool
WinCEBaroPort::Open()
{
  assert(!Thread::IsInside());

  // Open the Pressure port.
  switch(GetDeviceType()) 
  {
  case S3867: 
    pDeviceName = L"PSA1:";
    StoppableThread::Start();
    StateChanged();
	  break;
  case S3877: 
    pDeviceName = L"BAR1:";
    StoppableThread::Start();
    StateChanged();
	  break;
  case BikePilot:
    pDeviceName = L"BPA1:";
    StoppableThread::Start();
    StateChanged();
    break;
  default:
    pDeviceName = nullptr;
	  break;
  }

  return (pDeviceName == nullptr)? false : true;
}

int WinCEBaroPort::GetAtlitude(int pressure) {
	double rv;
	float A = pressure/ 101325.0f;
	float B = 1/5.25588f;
	rv = pow(A,B);
	rv = 1 - rv;
	rv = rv /0.0000225577;
	rv*=3.2808399; // feet
	return (int)rv;	
}

#define PSE_METHODE				0x00000000		//METHOD_BUFFERED
#define PSE_DEVICE_ACCE			0x00000000

#define S3867_PSE_DEVICE_TYPE	0x00008000
#define S3867_PSE_FUNCTION1		0x00000880		// Input 0 Output 4
#define S3867_PSE_FUNCTION2		0x00000881		// Input 0 Output 4

int WinCEBaroPort::GetPressureS3867() {
	int rc=101325;
	DWORD BytesReturned;
  HANDLE hPort = CreateFile(pDeviceName, GENERIC_READ|GENERIC_WRITE, 1, nullptr, OPEN_EXISTING, 0, nullptr);
  if(hPort != INVALID_HANDLE_VALUE) {
    BOOL retVal;
    DWORD err;
    BytesReturned=0;
    DWORD dataSize=sizeof(int);

    retVal = DeviceIoControl(hPort, CTL_CODE(S3867_PSE_DEVICE_TYPE, S3867_PSE_FUNCTION1, PSE_METHODE, PSE_DEVICE_ACCE),
                nullptr, 0, &rc, dataSize, &BytesReturned, nullptr);

    err = GetLastError();
    if(retVal == 0 || err != 0) {
      return 0;
    }

    retVal = DeviceIoControl(hPort, CTL_CODE(S3867_PSE_DEVICE_TYPE, S3867_PSE_FUNCTION2, PSE_METHODE, PSE_DEVICE_ACCE),
                nullptr, 0, &rc, dataSize, &BytesReturned, nullptr);

    err = GetLastError();
    if(retVal == 0 || err != 0) {
      return 0;
    }
    CloseHandle(hPort);
  }
	return rc;	
}


#if 0
#define S3877_PSE_DEVICE_TYPE 0x00000004
#define S3877_PSE_FUNCTION1   0x00000BBB    // Input 0 Output 4
#define S3877_PSE_FUNCTION2   0x00000BB8    // Input 0 Output 4


int WinCEBaroPort::GetPressureS3877() {
  int rc=101325;
  HANDLE hPort = CreateFile(pDeviceName, GENERIC_READ|GENERIC_WRITE, 1, nullptr, OPEN_EXISTING, 0, nullptr);
  if(hPort != INVALID_HANDLE_VALUE) {

    DWORD BytesReturned;
    BOOL retVal;
    DWORD err;
    BytesReturned=0;

    int data[4]={0,};
    DWORD dataSize=sizeof(data);

    retVal = DeviceIoControl(hPort, CTL_CODE(S3877_PSE_DEVICE_TYPE, S3877_PSE_FUNCTION1, PSE_METHODE, PSE_DEVICE_ACCE),
        nullptr,  0, data, dataSize, &BytesReturned, nullptr);


    err = GetLastError();
    if(retVal == 0 || err != 0) {
      return 0;
    }

    DeviceIoControl(hPort, CTL_CODE(S3877_PSE_DEVICE_TYPE, S3877_PSE_FUNCTION2, PSE_METHODE, PSE_DEVICE_ACCE),
        nullptr,  0, data, dataSize, &BytesReturned, nullptr);


    CloseHandle(hPort);
    rc=data[1];
  }

  return rc;

}

#else

typedef float (*barGetPressure_Type)();

int WinCEBaroPort::GetPressureS3877()
{
  int rc=101325;
  static HINSTANCE hDLL = nullptr;
  static DWORD error = 0;
  if(hDLL == nullptr && error == 0){
    hDLL = LoadLibrary(L"\\My Flash Disk\\MNAV\\HLXAPI.DLL");
    if(hDLL == nullptr) {
      error = GetLastError();
    }
  }

  if(hDLL != nullptr) {
    static barGetPressure_Type  barGetPressure = nullptr;
    if(barGetPressure == nullptr) {
      barGetPressure = (barGetPressure_Type) GetProcAddress(hDLL,L"barGetPressure");
    }
    float pressure = barGetPressure();
    pressure=pressure*100.f;
    rc= (int)pressure;
  }
  return rc;
}
#endif

typedef void (* YFPressureInit_t)();
typedef void (* YFPressureGetData_t)(DWORD &data);


int WinCEBaroPort::GetPressureBikePilot()
{
  int rc=101325;
  static HINSTANCE pressureApiDll = nullptr;
  static DWORD error = 0;

  static YFPressureInit_t pressureInit = nullptr;
  static YFPressureGetData_t pressureGetData = nullptr;
  if(pressureApiDll==nullptr && error == 0) {
    pressureApiDll = LoadLibrary(_T("YFPressureApi.dll"));
    if(pressureApiDll == nullptr) {
      error = GetLastError();
    } else {
      pressureInit = (YFPressureInit_t) GetProcAddress(pressureApiDll, _T("YFPressureInit"));
      if(pressureInit) {
        pressureGetData = (YFPressureGetData_t) GetProcAddress(pressureApiDll, _T("YFPressureGetData"));
        (*pressureInit)();
      } else {
        error = GetLastError();
      }
    }
  }
  if(pressureGetData) {
    DWORD data=0;
    (*pressureGetData)(data);
    rc = data;
  }
  return rc;
}



int WinCEBaroPort::GetPressure() {
  int rc = 101325;
  switch(GetDeviceType())
  {
  case S3867:
    rc = GetPressureS3867();
    break;
  case S3877:
    rc = GetPressureS3877();
    break;
  case BikePilot:
    rc = GetPressureBikePilot();
    break;
  default:
    break;
  }
  return rc;
}
	

char WinCEBaroPort::GetCheckSumm(const char *data) {
	const char *p=data;
	if(*p != '$')
		return 0;
	char rc = 0;
	while(*(++p) != '*') {
		if(*p == 0)
			return 0;
		rc ^= *p;
	}
	return rc;
}


DWORD WinCEBaroPort::CreatePressureNMEA(void *buffer, size_t length) {
	return snprintf((char*)buffer, length, "PRS %x\n", GetPressure());
} 


void WinCEBaroPort::Run()
{
  assert(Thread::IsInside());

  DWORD dwBytesTransferred;
  BYTE inbuf[1024];
  HANDLE hEvent = CreateEvent(nullptr, false, false, nullptr);


  // JMW added purging of port on open to prevent overflow
  Flush();
  while (!CheckStopped()) {
	if(WaitForSingleObject (hEvent, 50) != WAIT_TIMEOUT) {
      Sleep(50);
      continue;
	}
	dwBytesTransferred=CreatePressureNMEA(inbuf, sizeof(inbuf));
	handler.DataReceived(inbuf, dwBytesTransferred);
  }

  Flush();
}

PortState WinCEBaroPort::GetState() const {
  return (pDeviceName != nullptr)
    ? PortState::READY
    : PortState::FAILED;
}
