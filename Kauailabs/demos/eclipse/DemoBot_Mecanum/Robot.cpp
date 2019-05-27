/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/drive/MecanumDrive.h>
#include "AHRS.h"
using namespace frc;


class Robot : public TimedRobot {
public:
	Robot() {
		p_drive = new MecanumDrive(motor_left_front, motor_left_rear, motor_right_front, motor_right_rear);
		p_drive->SetRightSideInverted(true);
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() override {
	}

	void TeleopInit() override {}

	void TeleopPeriodic() override {
		double drive_strafe = -joy.GetX();
		double drive_fwd = -joy.GetY();
		double drive_rot = 0;
		if (joy.GetRawButton(6)) {
			drive_rot -= .8;
		}
		if (joy.GetRawButton(5)) {
			drive_rot += .8;
		}
		double gyro_angle = 0.0;
		p_drive->DriveCartesian(drive_strafe, drive_fwd, drive_rot, gyro_angle);
		/*
		wpi::outs() <<
				" Strafe:  " << drive_strafe <<
				" Fwd:  " << drive_fwd <<
				" Rot:  " << drive_rot <<
				" Gyro Angle:  " << gyro_angle <<
				" navX-Sensor Yaw:  " << ahrs.GetYaw() << "\n";
		*/
		/*
		motor_left_front.Set(drive_speed);
		motor_right_front.Set(drive_speed);
		motor_left_rear.Set(drive_speed);
		motor_right_rear.Set(drive_speed);
		int left_front_ticks = encoder_left_front.Get();
		int right_front_ticks = encoder_right_front.Get();
		int right_rear_ticks = encoder_right_rear.Get();
		int left_rear_ticks = encoder_left_rear.Get();
		wpi::outs() << "Joystick value:  " << drive_speed << " - encoders:  " <<
				left_front_ticks << " - " <<
				right_front_ticks << " - " <<
				right_rear_ticks << " - " <<
				left_rear_ticks << "\n";
		*/

	}

	void TestPeriodic() override {}

private:
	Joystick joy{0};
	Spark motor_left_front{0};
	Spark motor_right_front{1};
	Spark motor_right_rear{2};
	Spark motor_left_rear{3};
	Encoder encoder_left_front{0,1};
	Encoder encoder_right_front{2,3};
	Encoder encoder_right_rear{4,5};
	Encoder encoder_left_rear{6,7};
	MecanumDrive *p_drive;
	AHRS ahrs{50};
};

int main() {
	return frc::StartRobot<Robot>();
}
