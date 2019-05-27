/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <stdint.h>
#include <string>

#include "hal/Types.h"

HAL_ENUM(HAL_Mau_BoardAxis) {
	kHal_Mau_BoardAxisX = 0,
	kHal_Mau_BoardAxisY = 1,
	kHal_Mau_BoardAxisZ = 2,
};

struct HAL_Mau_BoardYawAxis
{
	/* Identifies one of the board axes */
	HAL_Mau_BoardAxis board_axis;
	/* true if axis is pointing up (with respect to gravity); false if pointing down. */
	bool up;
};

typedef struct HAL_BoardYawAxis HAL_BoardYawAxis;

#ifdef __cplusplus
extern "C" {
#endif

void	HAL_Mau_AHRS_Init(uint8_t update_rate_hz);
float	HAL_Mau_AHRS_GetPitch();
float	HAL_Mau_AHRS_GetRoll();
float	HAL_Mau_AHRS_GetYaw();
float	HAL_Mau_AHRS_GetCompassHeading();
void	HAL_Mau_AHRS_ZeroYaw();
bool	HAL_Mau_AHRS_IsCalibrating();
bool	HAL_Mau_AHRS_IsConnected();
double	HAL_Mau_AHRS_GetByteCount();
double	HAL_Mau_AHRS_GetUpdateCount();
long	HAL_Mau_AHRS_GetLastSensorTimestamp();
float	HAL_Mau_AHRS_GetWorldLinearAccelX();
float	HAL_Mau_AHRS_GetWorldLinearAccelY();
float	HAL_Mau_AHRS_GetWorldLinearAccelZ();
bool	HAL_Mau_AHRS_IsMoving();
bool	HAL_Mau_AHRS_IsRotating();
float	HAL_Mau_AHRS_GetBarometricPressure();
float	HAL_Mau_AHRS_GetAltitude();
bool	HAL_Mau_AHRS_IsAltitudeValid();
float	HAL_Mau_AHRS_GetFusedHeading();
bool	HAL_Mau_AHRS_IsMagneticDisturbance();
bool	HAL_Mau_AHRS_IsMagnetometerCalibrated();
float	HAL_Mau_AHRS_GetQuaternionW();
float	HAL_Mau_AHRS_GetQuaternionX();
float	HAL_Mau_AHRS_GetQuaternionY();
float	HAL_Mau_AHRS_GetQuaternionZ();
void	HAL_Mau_AHRS_ResetDisplacement();
void	HAL_Mau_AHRS_UpdateDisplacement( float accel_x_g, float accel_y_g,
                               		int update_rate_hz, bool is_moving );
float	HAL_Mau_AHRS_GetVelocityX();
float	HAL_Mau_AHRS_GetVelocityY();
float	HAL_Mau_AHRS_GetVelocityZ();
float	HAL_Mau_AHRS_GetDisplacementX();
float	HAL_Mau_AHRS_GetDisplacementY();
float	HAL_Mau_AHRS_GetDisplacementZ();
double	HAL_Mau_AHRS_GetAngle();
double	HAL_Mau_AHRS_GetRate();
void	HAL_Mau_AHRS_SetAngleAdjustment(double angle);
double	HAL_Mau_AHRS_GetAngleAdjustment();
void	HAL_Mau_AHRS_Reset();
float	HAL_Mau_AHRS_GetRawGyroX();
float	HAL_Mau_AHRS_GetRawGyroY();
float	HAL_Mau_AHRS_GetRawGyroZ();
float	HAL_Mau_AHRS_GetRawAccelX();
float	HAL_Mau_AHRS_GetRawAccelY();
float	HAL_Mau_AHRS_GetRawAccelZ();
float	HAL_Mau_AHRS_GetRawMagX();
float	HAL_Mau_AHRS_GetRawMagY();
float	HAL_Mau_AHRS_GetRawMagZ();
float	HAL_Mau_AHRS_GetPressure();
float	HAL_Mau_AHRS_GetTempC();
HAL_Mau_BoardYawAxis HAL_Mau_AHRS_GetBoardYawAxis();
const char * HAL_Mau_AHRS_GetFirmwareVersion();

// TODO:  Figure out how the callback (including JNI usage) should work.
//bool 	HAL_Mau_AHRS_RegisterCallback( IHAL_Mau_TimestampedDataSubscriber *callback, void *callback_context);
//bool 	HAL_Mau_AHRS_DeregisterCallback( IHAL_Mau_TimestampedDataSubscriber *callback );

int	HAL_Mau_AHRS_GetActualUpdateRate();
int	HAL_Mau_AHRS_GetRequestedUpdateRate();

void	HAL_Mau_AHRS_EnableLogging(bool enable);
void	HAL_Mau_AHRS_EnableBoardlevelYawReset(bool enable);
bool	HAL_Mau_AHRS_IsBoardlevelYawResetEnabled();

int16_t	HAL_Mau_AHRS_GetGyroFullScaleRangeDPS();
int16_t HAL_Mau_AHRS_GetAccelFullScaleRangeG();

#ifdef __cplusplus
}  // extern "C"
/** @} */
#endif
