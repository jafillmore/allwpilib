package com.kauailabs.vmx.wpilib.Examples.DemoBot_Mecanum_AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {

	Joystick joy;
	Spark motor_left_front;
	Spark motor_right_front;
	Spark motor_right_rear;
	Spark motor_left_rear;
	Encoder encoder_left_front;
	Encoder encoder_right_front;
	Encoder encoder_right_rear;
	Encoder encoder_left_rear;
	AHRS ahrs;
	MecanumDrive drive;
 
	public Robot() {
		joy = new Joystick(0);
		motor_left_front = new Spark(0);
		motor_right_front = new Spark(1);
		motor_right_rear = new Spark(2);
		motor_left_rear = new Spark(3);
		encoder_left_front = new Encoder(0,1);
		encoder_right_front = new Encoder(2,3);
		encoder_right_rear = new Encoder(4,5);
		encoder_left_rear = new Encoder(6,7);
		ahrs = new AHRS(SPI.Port.kMXP,(byte)50);
		drive = new MecanumDrive(motor_left_front, motor_left_rear, motor_right_front, motor_right_rear);
		drive.setRightSideInverted(true);

	}

	public void robotInit() {
		System.out.println("WPI Library Java DemoBot (Mecanum) Program");
	}

	public void teleopPeriodic() {
		System.out.println("teleopPeriodic");
		double drive_strafe = -joy.getX();
		double drive_fwd = -joy.getY();
		double drive_rot = 0;
		if (joy.getRawButton(6)) {
			drive_rot -= .8;
		}
		if (joy.getRawButton(5)) {
			drive_rot += .8;
		}
		
		float gyro_angle = ahrs.getYaw();
		drive.driveCartesian(drive_strafe, drive_fwd, drive_rot, gyro_angle);					
		System.out.println("Strafe:  " + drive_strafe + " Fwd:  " + 
			drive_fwd + " Rot:  " +
			drive_rot + " Gyro Angle:  " +
			gyro_angle + " navX-Sensor Yaw:  " +
			ahrs.getYaw());		
		
	}

	public void disabled() {
	}
}
