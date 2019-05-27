/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../include/AHRS_Mau.h"

#include <AHRS.h>
#include <VMXPi.h>

static AHRS *p_ahrs = 0;

void	HAL_Mau_AHRS_Init(uint8_t update_rate_hz)
{
	VMXPi *vmx_pi = VMXPi::getInstance();
	p_ahrs = &(vmx_pi->getAHRS());
}

float	HAL_Mau_AHRS_GetPitch()
{
	return p_ahrs->GetPitch();
}

float	HAL_Mau_AHRS_GetRoll()
{
	return p_ahrs->GetRoll();
}

float	HAL_Mau_AHRS_GetYaw()
{
	return p_ahrs->GetYaw();
}

float	HAL_Mau_AHRS_GetCompassHeading()
{
	return p_ahrs->GetCompassHeading();
}

void	HAL_Mau_AHRS_ZeroYaw()
{
	p_ahrs->ZeroYaw();
}

bool	HAL_Mau_AHRS_IsCalibrating()
{
	return p_ahrs->IsCalibrating();
}

bool	HAL_Mau_AHRS_IsConnected()
{
	return p_ahrs->IsConnected();
}

double	HAL_Mau_AHRS_GetByteCount()
{
	return p_ahrs->GetByteCount();
}

double	HAL_Mau_AHRS_GetUpdateCount()
{
	return p_ahrs->GetUpdateCount();
}

long	HAL_Mau_AHRS_GetLastSensorTimestamp()
{
	return p_ahrs->GetLastSensorTimestamp();
}

float	HAL_Mau_AHRS_GetWorldLinearAccelX()
{
	return p_ahrs->GetWorldLinearAccelX();
}

float	HAL_Mau_AHRS_GetWorldLinearAccelY()
{
	return p_ahrs->GetWorldLinearAccelY();
}

float	HAL_Mau_AHRS_GetWorldLinearAccelZ()
{
	return p_ahrs->GetWorldLinearAccelZ();
}

bool	HAL_Mau_AHRS_IsMoving()
{
	return p_ahrs->IsMoving();
}

bool	HAL_Mau_AHRS_IsRotating()
{
	return p_ahrs->IsRotating();
}

float	HAL_Mau_AHRS_GetBarometricPressure()
{
	return p_ahrs->GetBarometricPressure();
}

float	HAL_Mau_AHRS_GetAltitude()
{
	return p_ahrs->GetAltitude();
}

bool	HAL_Mau_AHRS_IsAltitudeValid()
{
	return p_ahrs->IsAltitudeValid();
}

float	HAL_Mau_AHRS_GetFusedHeading()
{
	return p_ahrs->GetFusedHeading();
}

bool	HAL_Mau_AHRS_IsMagneticDisturbance()
{
	return p_ahrs->IsMagneticDisturbance();
}

bool	HAL_Mau_AHRS_IsMagnetometerCalibrated()
{
	return p_ahrs->IsMagnetometerCalibrated();
}

float	HAL_Mau_AHRS_GetQuaternionW()
{
	return p_ahrs->GetQuaternionW();
}

float	HAL_Mau_AHRS_GetQuaternionX()
{
	return p_ahrs->GetQuaternionX();
}

float	HAL_Mau_AHRS_GetQuaternionY()
{
	return p_ahrs->GetQuaternionY();
}

float	HAL_Mau_AHRS_GetQuaternionZ()
{
	return p_ahrs->GetQuaternionZ();
}

void	HAL_Mau_AHRS_ResetDisplacement()
{
	p_ahrs->ResetDisplacement();
}

void	HAL_Mau_AHRS_UpdateDisplacement( float accel_x_g, float accel_y_g,
                               		int update_rate_hz, bool is_moving )
{
	p_ahrs->UpdateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
}

float	HAL_Mau_AHRS_GetVelocityX()
{
	return p_ahrs->GetVelocityX();
}

float	HAL_Mau_AHRS_GetVelocityY()
{
	return p_ahrs->GetVelocityY();
}

float	HAL_Mau_AHRS_GetVelocityZ()
{
	return p_ahrs->GetVelocityZ();
}

float	HAL_Mau_AHRS_GetDisplacementX()
{
	return p_ahrs->GetDisplacementX();
}

float	HAL_Mau_AHRS_GetDisplacementY()
{
	return p_ahrs->GetDisplacementY();
}

float	HAL_Mau_AHRS_GetDisplacementZ()
{
	return p_ahrs->GetDisplacementZ();
}

double	HAL_Mau_AHRS_GetAngle()
{
	return p_ahrs->GetAngle();
}

double	HAL_Mau_AHRS_GetRate()
{
	return p_ahrs->GetRate();
}

void	HAL_Mau_AHRS_SetAngleAdjustment(double angle)
{
	// TODO:  Angle Adjustments not currently supported in VMX-pi AHRS class
	//p_ahrs->SetAngleAdjustment(angle);
}

double	HAL_Mau_AHRS_GetAngleAdjustment()
{
	// TODO:  Angle Adjustments not currently supported in VMX-pi AHRS class
	//return p_ahrs->GetAngleAdjustment();
	return 0.0;
}

void	HAL_Mau_AHRS_Reset()
{
	p_ahrs->Reset();
}

float	HAL_Mau_AHRS_GetRawGyroX()
{
	return p_ahrs->GetRawGyroX();
}

float	HAL_Mau_AHRS_GetRawGyroY()
{
	return p_ahrs->GetRawGyroY();
}

float	HAL_Mau_AHRS_GetRawGyroZ()
{
	return p_ahrs->GetRawGyroZ();
}

float	HAL_Mau_AHRS_GetRawAccelX()
{
	return p_ahrs->GetRawAccelX();
}

float	HAL_Mau_AHRS_GetRawAccelY()
{
	return p_ahrs->GetRawAccelY();
}

float	HAL_Mau_AHRS_GetRawAccelZ()
{
	return p_ahrs->GetRawAccelZ();
}

float	HAL_Mau_AHRS_GetRawMagX()
{
	return p_ahrs->GetRawMagX();
}

float	HAL_Mau_AHRS_GetRawMagY()
{
	return p_ahrs->GetRawMagY();
}

float	HAL_Mau_AHRS_GetRawMagZ()
{
	return p_ahrs->GetRawMagZ();
}

float	HAL_Mau_AHRS_GetPressure()
{
	return p_ahrs->GetPressure();
}

float	HAL_Mau_AHRS_GetTempC()
{
	return p_ahrs->GetTempC();
}

HAL_Mau_BoardYawAxis HAL_Mau_AHRS_GetBoardYawAxis()
{
	HAL_Mau_BoardYawAxis mau_board_axis;
	::BoardYawAxis axis = p_ahrs->GetBoardYawAxis();
	switch (axis.board_axis) {
	case ::BoardAxis::kBoardAxisX:
		mau_board_axis.board_axis = HAL_Mau_BoardAxis::kHal_Mau_BoardAxisX;
		break;
	case ::BoardAxis::kBoardAxisY:
		mau_board_axis.board_axis = HAL_Mau_BoardAxis::kHal_Mau_BoardAxisY;
		break;
	case ::BoardAxis::kBoardAxisZ:
	default:
		mau_board_axis.board_axis = HAL_Mau_BoardAxis::kHal_Mau_BoardAxisZ;
		break;
	}
	mau_board_axis.up = axis.up;
	return mau_board_axis;
}

const char * HAL_Mau_AHRS_GetFirmwareVersion()
{
	return p_ahrs->GetFirmwareVersion().c_str();
}

// TODO:  Figure out how the callback (including JNI usage) should work.
//bool 	HAL_Mau_AHRS_RegisterCallback( IHAL_Mau_TimestampedDataSubscriber *callback, void *callback_context);
//bool 	HAL_Mau_AHRS_DeregisterCallback( IHAL_Mau_TimestampedDataSubscriber *callback );

int	HAL_Mau_AHRS_GetActualUpdateRate()
{
	return p_ahrs->GetActualUpdateRate();
}

int	HAL_Mau_AHRS_GetRequestedUpdateRate()
{
	return p_ahrs->GetRequestedUpdateRate();
}

void	HAL_Mau_AHRS_EnableLogging(bool enable)
{
	// TODO:  Underlying VMX-pi AHRS class does not support logging currently.
	//p_ahrs->EnableLogging();
}

void	HAL_Mau_AHRS_EnableBoardlevelYawReset(bool enable)
{
	// TODO:  Underlying VMX-pi AHRS class does not support disabling of board-level yaw resets.
	//p_ahrs->EnableBoardlevelYawReset(enable);
}

bool	HAL_Mau_AHRS_IsBoardlevelYawResetEnabled()
{
	// TODO:  Underlying VMX-pi AHRS class does not support disabling of board-level yaw resets.
	//return p_ahrs->IsBoardlevelYawResetEnabled();
	return true;
}

int16_t	HAL_Mau_AHRS_GetGyroFullScaleRangeDPS()
{
	// TODO:  Update AHRS class so this is no longer hard-coded.
	return 2000;
}

int16_t HAL_Mau_AHRS_GetAccelFullScaleRangeG()
{
	// TODO:  Update AHRS class so this is no longer hard-coded.
	return 2;
}
