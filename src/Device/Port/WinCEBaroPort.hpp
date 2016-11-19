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

#ifndef XCSOAR_DEVICE_S3768_PORT_HPP
#define XCSOAR_DEVICE_S3768_PORT_HPP


#include "Thread/StoppableThread.hpp"
#include "IO/DataHandler.hpp"
#include "Port.hpp"
#include "Math/KalmanFilter1d.hpp"


#include <windef.h>
#include <winioctl.h>
#include <sstream>
#include <vector>

enum WinCE_DeviceType {
	S3867,
	S3877,
	BikePilot,
	UNKNON
};


/**
 * Generic WinCEBaroPort thread handler class
 */
class WinCEBaroPort : public Port, protected StoppableThread
{
  const wchar_t *pDeviceName;
  int GetPressure();
  int GetPressureS3867();
  int GetPressureS3877();
  int GetPressureBikePilot();
  
  
  int GetAtlitude(int pressure);
  char GetCheckSumm(const char *data);
  DWORD CreatePressureNMEA(void *buffer, size_t length);
  
  

public:
  /**
   * Creates a new serial port (RS-232) object, but does not open it yet.
   *
   * @param _handler the callback object for input received on the
   * port
   */
  WinCEBaroPort(PortListener *_listener, DataHandler &_handler);

  /**
   * Closes the serial port (Destructor)
   */
  virtual ~WinCEBaroPort();

  /**
   *	GetDeviceType();
   */ 
  static  WinCE_DeviceType GetDeviceType();
 
  
  /**
   * Opens the serial port
   * @return True on success, False on failure
   */
  bool Open();

protected:

public:

  /* virtual methods from class Port */
  PortState GetState() const override;
  size_t Write(const void *data, size_t length) override
  {return 0;}
  bool Drain() override
  {return true;}
  void Flush() override
  {}
  unsigned GetBaudrate() const override
  {return 0;}
  bool SetBaudrate(unsigned baud_rate) override
  {return true;}
  bool StopRxThread() override
  {return true;}
  bool StartRxThread() override
  {return true;}
  int Read(void *Buffer, size_t Size) override
  {return -1;}
  WaitResult WaitRead(unsigned timeout_ms) override
  {return WaitResult::FAILED;}
  
  
protected:
  /* virtual methods from class Thread */
  virtual void Run() override;
};

#endif
