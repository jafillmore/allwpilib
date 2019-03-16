/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/SerialPort.h"
#include "OSSerialPort.h"

#include "HALInitializer.h"

namespace hal {
    namespace init {
        void InitializeSerialPort() {}
    }
}

// All VMX-pi serial ports are accessible directly from Linux; therefore all of the HAL Serial
// Port calls map one-to-on onto the HAL_OSSerialPort interface.

extern "C" {
    void HAL_InitializeSerialPort(HAL_SerialPort port, int32_t* status) {
        hal::init::CheckInit();
        HAL_InitializeOSSerialPort(port, status);
    }

    void HAL_InitializeSerialPortDirect(HAL_SerialPort port, const char* portName, int32_t* status) {
    	HAL_InitializeSerialPort(port, status);
    }

    void HAL_SetSerialBaudRate(HAL_SerialPort port, int32_t baud, int32_t* status) {
    	HAL_SetOSSerialBaudRate(port, baud, status);
    }

    void HAL_SetSerialDataBits(HAL_SerialPort port, int32_t bits, int32_t* status) {
    	HAL_SetOSSerialDataBits(port, bits, status);
    }

    void HAL_SetSerialParity(HAL_SerialPort port, int32_t parity, int32_t* status) {
    	HAL_SetOSSerialParity(port, parity, status);
    }

    void HAL_SetSerialStopBits(HAL_SerialPort port, int32_t stopBits, int32_t* status) {
    	HAL_SetOSSerialStopBits(port, stopBits, status);
    }

    void HAL_SetSerialWriteMode(HAL_SerialPort port, int32_t mode, int32_t* status) {
    	HAL_SetOSSerialWriteMode(port, mode, status);
    }

    void HAL_SetSerialFlowControl(HAL_SerialPort port, int32_t flow, int32_t* status) {
    	HAL_SetOSSerialFlowControl(port, flow, status);
    }

    void HAL_SetSerialTimeout(HAL_SerialPort port, double timeout, int32_t* status) {
    	HAL_SetOSSerialTimeout(port, timeout, status);
    }

    void HAL_EnableSerialTermination(HAL_SerialPort port, char terminator, int32_t* status) {
    	HAL_EnableOSSerialTermination(port, terminator, status);
    }

    void HAL_DisableSerialTermination(HAL_SerialPort port, int32_t* status) {
    	HAL_DisableOSSerialTermination(port, status);
    }

    void HAL_SetSerialReadBufferSize(HAL_SerialPort port, int32_t size, int32_t* status) {
    	HAL_SetOSSerialReadBufferSize(port, size, status);
    }

    void HAL_SetSerialWriteBufferSize(HAL_SerialPort port, int32_t size, int32_t* status) {
    	HAL_SetOSSerialWriteBufferSize(port, size, status);
    }

    int32_t HAL_GetSerialBytesReceived(HAL_SerialPort port, int32_t* status) {
        return HAL_GetOSSerialBytesReceived(port, status);
    }

    int32_t HAL_ReadSerial(HAL_SerialPort port, char* buffer, int32_t count, int32_t* status) {
        return HAL_ReadOSSerial(port, buffer, count, status);
    }

    int32_t HAL_WriteSerial(HAL_SerialPort port, const char* buffer, int32_t count, int32_t* status) {
        return HAL_WriteOSSerial(port, buffer, count, status);
    }

    void HAL_FlushSerial(HAL_SerialPort port, int32_t* status) {
    	HAL_FlushOSSerial(port, status);
    }

    void HAL_ClearSerial(HAL_SerialPort port, int32_t* status) {
    	HAL_ClearOSSerial(port, status);
    }

    void HAL_CloseSerial(HAL_SerialPort port, int32_t* status) {
    	HAL_CloseOSSerial(port, status);
    }
}
