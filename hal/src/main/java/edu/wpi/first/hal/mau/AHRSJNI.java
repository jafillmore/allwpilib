/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.hal.mau;

import edu.wpi.first.hal.JNIWrapper;

public class AHRSJNI extends JNIWrapper {
  public static native void	Init(int update_rate_hz);

  public static native float	GetPitch();
  public static native float	GetRoll();
  public static native float	GetYaw();
  public static native float	GetCompassHeading();
  public static native void	ZeroYaw();
  public static native boolean 	IsCalibrating();
  public static native boolean	IsConnected();
  public static native double	GetByteCount();
  public static native double	GetUpdateCount();
  public static native int	GetLastSensorTimestamp();
  public static native float	GetWorldLinearAccelX();
  public static native float	GetWorldLinearAccelY();
  public static native float	GetWorldLinearAccelZ();
  public static native boolean 	IsMoving();
  public static native boolean 	IsRotating();
  public static native float	GetBarometricPressure();
  public static native float	GetAltitude();
  public static native boolean 	IsAltitudeValid();
  public static native float	GetFusedHeading();
  public static native boolean 	IsMagneticDisturbance();
  public static native boolean 	IsMagnetometerCalibrated();
  public static native float	GetQuaternionW();
  public static native float	GetQuaternionX();
  public static native float	GetQuaternionY();
  public static native float	GetQuaternionZ();
  public static native void	ResetDisplacement();
  public static native void	UpdateDisplacement( float accel_x_g, float accel_y_g,
                               		int update_rate_hz, boolean is_moving );
  public static native float	GetVelocityX();
  public static native float	GetVelocityY();
  public static native float	GetVelocityZ();
  public static native float	GetDisplacementX();
  public static native float	GetDisplacementY();
  public static native float	GetDisplacementZ();
  public static native double	GetAngle();
  public static native double	GetRate();
  public static native void 	SetAngleAdjustment(double angle);
  public static native double	GetAngleAdjustment();
  public static native void 	Reset();
  public static native float	GetRawGyroX();
  public static native float	GetRawGyroY();
  public static native float	GetRawGyroZ();
  public static native float	GetRawAccelX();
  public static native float	GetRawAccelY();
  public static native float	GetRawAccelZ();
  public static native float	GetRawMagX();
  public static native float	GetRawMagY();
  public static native float	GetRawMagZ();
  public static native float	GetPressure();
  public static native float	GetTempC();

  public static native int GetBoardYawAxisID();
  public static native boolean GetBoardYawAxisUp();
  public static native String	GetFirmwareVersion();

  // TODO:  Figure out how the callback (including JNI usage) should work.
  //bool 	HAL_Mau_RegisterCallback( IHAL_Mau_TimestampedDataSubscriber *callback, void *callback_context);
  //bool 	HAL_Mau_DeregisterCallback( IHAL_Mau_TimestampedDataSubscriber *callback );

  public static native int	GetActualUpdateRate();
  public static native int	GetRequestedUpdateRate();

  public static native void	EnableLogging(boolean enable);
  public static native void	EnableBoardlevelYawReset(boolean enable);
  public static native boolean	IsBoardlevelYawResetEnabled();

  public static native int	GetGyroFullScaleRangeDPS();

  public static native int	GetAccelFullScaleRangeG();
}
