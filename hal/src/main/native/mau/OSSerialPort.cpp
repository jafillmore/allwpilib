/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OSSerialPort.h"

#include <stdlib.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <iostream>
#include <list>
#include <chrono>
#include <cstring>
#include <string>

#include "hal/Errors.h"
#include "hal/cpp/SerialHelper.h"
#include "HALInitializer.h"
#include "RaspberryPiInfo.h"

#define INVALID_OS_SERIAL_PORT_INDEX -1

static int portHandles[4]{-1, -1, -1, -1};
static std::chrono::milliseconds portTimeouts[4]{
    std::chrono::milliseconds(0), std::chrono::milliseconds(0),
    std::chrono::milliseconds(0), std::chrono::milliseconds(0)};

namespace hal {
namespace init {
void InitializeOSSerialPort() {
  for (int i = 0; i < 4; i++) {
    portHandles[i] = -1;
    portTimeouts[i] = std::chrono::milliseconds(0);
  }
}

int GetOSSerialPortIndex(int handle) {
  for (int i = 0; i < 4; i++) {
    if (portHandles[i] == handle) {
      return i;
    }
  }  
  return INVALID_OS_SERIAL_PORT_INDEX;
}
}  // namespace init
}  // namespace hal

/* On VMX-pi, both Onboard and MXP Serial ports alias to the same underlying Raspberry Pi Serial port. */
constexpr const char* OnboardResourceOS = "/dev/ttyS0";
constexpr const char* MxpResourceOS = "/dev/ttyS0";

std::string GetOSSerialPortName(HAL_SerialPort port,
                                              int32_t* status) {
  if (port == HAL_SerialPort::HAL_SerialPort_Onboard) {
    return OnboardResourceOS;
  } else if (port == HAL_SerialPort::HAL_SerialPort_MXP) {
    return MxpResourceOS;
  }

  RaspberryPiDeviceInfo rpi_info = getRaspberryPiDeviceInfo();
  std::list<USBSerialDeviceInfo> rpi_usb_serial_devices = getRaspberryPiUSBSerialDeviceInfoList();

  // If no additional serial ports are found, return error
  if (rpi_usb_serial_devices.size() < 1) {
    *status = HAL_SERIAL_PORT_NOT_FOUND;
    return "";
  }

  std::list<USBSerialDeviceInfo>::iterator itf = rpi_usb_serial_devices.begin();
  while (itf != rpi_usb_serial_devices.end()) {
	  if (port == HAL_SerialPort_USB1) {
		  if ((*itf).physical_usb_port == RaspberryPiUSBPort::RPI_USB_PORTA) {
			  return (*itf).devname;
		  }
	  } else if (port == HAL_SerialPort_USB2) {
		  if ((*itf).physical_usb_port == RaspberryPiUSBPort::RPI_USB_PORTB) {
			  return (*itf).devname;
		  }
	  }
      itf++;
  }

  *status = HAL_SERIAL_PORT_NOT_FOUND;
  return "";
}



extern "C" {

HAL_SerialPortHandle HAL_InitializeOSSerialPort(HAL_SerialPort port, int32_t* status) {
  hal::init::CheckInit();
  std::string portName;

  portName = GetOSSerialPortName(port, status);

  if (*status < 0) {
    return HAL_kInvalidHandle;
  }

  int fs = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fs == -1) {
    *status = HAL_SERIAL_PORT_OPEN_ERROR;
    return HAL_kInvalidHandle;
  }
  portHandles[port] = fs;

  struct termios options;
  tcgetattr(fs, &options);
  options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(fs, TCIFLUSH);
  tcsetattr(fs, TCSANOW, &options);

  return portHandles[port];
}

void HAL_SetOSSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud,
                             int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  int baudRate = -1;
  switch (baud) {
    case 9600:
      baudRate = B9600;
      break;
    case 19200:
      baudRate = B19200;
      break;
    case 38400:
      baudRate = B38400;
      break;
    case 57600:
      baudRate = B57600;
      break;
    case 115200:
      baudRate = B115200;
      break;
    default:
      *status = PARAMETER_OUT_OF_RANGE;
      return;
  }

  struct termios options;
  tcgetattr(portHandles[port], &options);
  auto set = cfsetospeed(&options, baudRate);
  if (set != 0) {
    *status = HAL_SERIAL_PORT_ERROR;
    return;
  }
  set = tcsetattr(portHandles[port], TCSANOW, &options);
  if (set != 0) {
    *status = HAL_SERIAL_PORT_ERROR;
    return;
  }
}

void HAL_SetOSSerialDataBits(HAL_SerialPortHandle handle, int32_t bits,
                             int32_t* status) {
  int numBits = -1;
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  switch (bits) {
    case 5:
      numBits = CS5;
      break;
    case 6:
      numBits = CS6;
      break;
    case 7:
      numBits = CS7;
      break;
    case 8:
      numBits = CS8;
      break;
    default:
      *status = PARAMETER_OUT_OF_RANGE;
      return;
  }

  struct termios options;
  tcgetattr(portHandles[port], &options);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= numBits;
  auto set = tcsetattr(portHandles[port], TCSANOW, &options);
  if (set != 0) {
    *status = HAL_SERIAL_PORT_ERROR;
    return;
  }
}

void HAL_SetOSSerialParity(HAL_SerialPortHandle handle, int32_t parity,
                           int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }

  // Just set none parity
  struct termios options;
  tcgetattr(portHandles[port], &options);
  options.c_cflag &= ~PARENB;
  auto set = tcsetattr(portHandles[port], TCSANOW, &options);
  if (set != 0) {
    *status = HAL_SERIAL_PORT_ERROR;
    return;
  }
}

void HAL_SetOSSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits,
                             int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  // Force 1 stop bit
  struct termios options;
  tcgetattr(portHandles[port], &options);
  options.c_cflag &= ~CSTOPB;
  auto set = tcsetattr(portHandles[port], TCSANOW, &options);
  if (set != 0) {
    *status = HAL_SERIAL_PORT_ERROR;
    return;
  }
}

void HAL_SetOSSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode,
                              int32_t* status) {
  // No op
}

void HAL_SetOSSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow,
                                int32_t* status) {
  // No op
}

void HAL_SetOSSerialTimeout(HAL_SerialPortHandle handle, double timeout,
                            int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  // Convert to millis
  int t = timeout / 1000;
  portTimeouts[port] = std::chrono::milliseconds(t);
}

void HAL_EnableOSSerialTermination(HAL_SerialPortHandle handle, char terminator,
                                   int32_t* status) {
  // \n is hardcoded for now. Will fix later
  // Seems like a VISA only setting, need to check
}

void HAL_DisableOSSerialTermination(HAL_SerialPortHandle handle, int32_t* status) {
  // Seems like a VISA only setting, need to check
}

void HAL_SetOSSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size,
                                   int32_t* status) {
  // No op
}

void HAL_SetOSSerialWriteBufferSize(HAL_SerialPortHandle handle, int32_t size,
                                    int32_t* status) {
  // No op
}

int32_t HAL_GetOSSerialBytesReceived(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }
  int bytes = 0;
  ioctl(portHandles[port], FIONREAD, &bytes);
  return bytes;
}

int32_t HAL_ReadOSSerial(HAL_SerialPortHandle handle, char* buffer, int32_t count,
                         int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }

  auto endTime = std::chrono::steady_clock::now() + portTimeouts[port];

  int bytesRead = 0;

  unsigned char buf[256];

  do {
    int rx = read(portHandles[port], buf, count - bytesRead);
    std::memcpy(&buffer[bytesRead], buf, rx);
    bytesRead += rx;
    if (bytesRead >= count) break;
    wpi::StringRef tmp(buffer, bytesRead);
    auto loc = tmp.find('\n');
    if (loc != wpi::StringRef::npos) {
      bytesRead = loc;
      break;
    }
  } while (std::chrono::steady_clock::now() < endTime);
  return bytesRead;
}

int32_t HAL_WriteOSSerial(HAL_SerialPortHandle handle, const char* buffer,
                          int32_t count, int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return 0;
  }
  return write(portHandles[port], buffer, count);
}
void HAL_FlushOSSerial(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  tcdrain(portHandles[port]);
}
void HAL_ClearOSSerial(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  tcflush(portHandles[port], TCIOFLUSH);
}
void HAL_CloseOSSerial(HAL_SerialPortHandle handle, int32_t* status) {
  auto port = hal::init::GetOSSerialPortIndex(handle);
  if (port == INVALID_OS_SERIAL_PORT_INDEX) {
    *status = HAL_HANDLE_ERROR;
    return;
  }
  close(portHandles[port]);
}

}  // extern "C"
