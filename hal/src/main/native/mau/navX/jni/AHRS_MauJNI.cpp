/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <jni.h>

#include "edu_wpi_first_hal_mau_AHRSJNI.h"
#include "../include/AHRS_Mau.h"
#include <wpi/jni_util.h>

#include "hal/Types.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    Init
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_Init
  (JNIEnv *, jclass, jint update_rate_hz)
{
	HAL_Mau_AHRS_Init(update_rate_hz);
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetPitch
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetPitch
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetPitch();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRoll
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRoll
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRoll();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetYaw
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetYaw
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetYaw();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetCompassHeading
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetCompassHeading
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetCompassHeading();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    ZeroYaw
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_ZeroYaw
  (JNIEnv *, jclass)
{
	HAL_Mau_AHRS_ZeroYaw();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsCalibrating
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsCalibrating
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsCalibrating();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsConnected
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsConnected
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsConnected();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetByteCount
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetByteCount
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetByteCount();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetUpdateCount
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetUpdateCount
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetUpdateCount();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetLastSensorTimestamp
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetLastSensorTimestamp
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetLastSensorTimestamp();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetWorldLinearAccelX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetWorldLinearAccelX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetWorldLinearAccelX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetWorldLinearAccelY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetWorldLinearAccelY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetWorldLinearAccelY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetWorldLinearAccelZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetWorldLinearAccelZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetWorldLinearAccelZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsMoving
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsMoving
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsMoving();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsRotating
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsRotating
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsRotating();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetBarometricPressure
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetBarometricPressure
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetBarometricPressure();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetAltitude
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetAltitude
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetAltitude();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsAltitudeValid
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsAltitudeValid
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsAltitudeValid();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetFusedHeading
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetFusedHeading
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetFusedHeading();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsMagneticDisturbance
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsMagneticDisturbance
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsMagneticDisturbance();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsMagnetometerCalibrated
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsMagnetometerCalibrated
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsMagnetometerCalibrated();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetQuaternionW
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetQuaternionW
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetQuaternionW();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetQuaternionX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetQuaternionX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetQuaternionX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetQuaternionY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetQuaternionY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetQuaternionY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetQuaternionZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetQuaternionZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetQuaternionZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    ResetDisplacement
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_ResetDisplacement
  (JNIEnv *, jclass)
{
	HAL_Mau_AHRS_ResetDisplacement();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    UpdateDisplacement
 * Signature: (FFIZ)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_UpdateDisplacement
  (JNIEnv *, jclass, jfloat accel_x_g, jfloat accel_y_g, jint update_rate_hz, jboolean is_moving)
{
	HAL_Mau_AHRS_UpdateDisplacement(accel_x_g, accel_y_g, update_rate_hz, is_moving);
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetVelocityX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetVelocityX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetVelocityX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetVelocityY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetVelocityY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetVelocityY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetVelocityZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetVelocityZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetVelocityZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetDisplacementX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetDisplacementX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetDisplacementX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetDisplacementY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetDisplacementY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetDisplacementY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetDisplacementZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetDisplacementZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetDisplacementZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetAngle
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetAngle
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetAngle();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRate
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRate
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRate();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    SetAngleAdjustment
 * Signature: (D)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_SetAngleAdjustment
  (JNIEnv *, jclass, jdouble angle)
{
	HAL_Mau_AHRS_SetAngleAdjustment(angle);
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetAngleAdjustment
 * Signature: ()D
 */
JNIEXPORT jdouble JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetAngleAdjustment
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetAngleAdjustment();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    Reset
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_Reset
  (JNIEnv *, jclass)
{
	HAL_Mau_AHRS_Reset();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawGyroX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawGyroX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawGyroX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawGyroY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawGyroY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawGyroY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawGyroZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawGyroZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawGyroZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawAccelX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawAccelX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawAccelX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawAccelY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawAccelY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawAccelY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawAccelZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawAccelZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawAccelZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawMagX
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawMagX
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawMagX();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawMagY
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawMagY
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawMagY();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRawMagZ
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRawMagZ
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRawMagZ();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetPressure
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetPressure
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetPressure();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetTempC
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetTempC
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetTempC();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetBoardYawAxisID
 * Signature: ()Ledu/wpi/first/hal/mau/BoardYawAxis;
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetBoardYawAxisID
  (JNIEnv *, jclass)
{	
	HAL_Mau_BoardYawAxis board_yaw_axis = HAL_Mau_AHRS_GetBoardYawAxis();
	int axis_id = (int)board_yaw_axis.board_axis;
	return axis_id;
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetBoardYawAxisID
 * Signature: ()Ledu/wpi/first/hal/mau/BoardYawAxis;
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetBoardYawAxisUp
  (JNIEnv *, jclass)
{	
	HAL_Mau_BoardYawAxis board_yaw_axis = HAL_Mau_AHRS_GetBoardYawAxis();
	bool up = board_yaw_axis.up;
	return up;
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetFirmwareVersion
 * Signature: ()Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetFirmwareVersion
  (JNIEnv *env, jclass)
{
	return MakeJString(env, HAL_Mau_AHRS_GetFirmwareVersion());
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetActualUpdateRate
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetActualUpdateRate
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetActualUpdateRate();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetRequestedUpdateRate
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetRequestedUpdateRate
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetRequestedUpdateRate();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    EnableLogging
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_EnableLogging
  (JNIEnv *, jclass, jboolean enable)
{
	HAL_Mau_AHRS_EnableLogging(enable);
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    EnableBoardlevelYawReset
 * Signature: (Z)V
 */
JNIEXPORT void JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_EnableBoardlevelYawReset
  (JNIEnv *, jclass, jboolean enable)
{
	HAL_Mau_AHRS_EnableBoardlevelYawReset(enable);
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    IsBoardlevelYawResetEnabled
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_IsBoardlevelYawResetEnabled
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_IsBoardlevelYawResetEnabled();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetGyroFullScaleRangeDPS
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetGyroFullScaleRangeDPS
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetGyroFullScaleRangeDPS();
}

/*
 * Class:     edu_wpi_first_hal_mau_AHRSJNI
 * Method:    GetAccelFullScaleRangeG
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_edu_wpi_first_hal_mau_AHRSJNI_GetAccelFullScaleRangeG
  (JNIEnv *, jclass)
{
	return HAL_Mau_AHRS_GetAccelFullScaleRangeG();
}

}  // extern "C"
