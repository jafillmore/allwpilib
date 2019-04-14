package com.kauailabs.vmx.wpilib.Examples.DemoBot_Mecanum;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

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
	// TODO:  AHRS

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
	}

	public void robotInit() {
		System.out.println("WPI Library Java DemoBot (Mecanum) Program");
	}

	public void teleopPeriodic() {
		double drive_speed = joy.getX();
		motor_left_front.set(drive_speed);
		motor_right_front.set(drive_speed);
		motor_left_rear.set(drive_speed);
		motor_right_rear.set(drive_speed);						
		int left_front_ticks = encoder_left_front.get();
		int right_front_ticks = encoder_right_front.get();
		int right_rear_ticks = encoder_right_rear.get();
		int left_rear_ticks = encoder_left_rear.get();						
		System.out.println("Joystick value:  " + drive_speed + " - encoders:  " + 
			left_front_ticks + " - " +
			right_front_ticks + " - " +
			right_rear_ticks + " - " +
			left_rear_ticks);						
	}

	public void disabled() {
	}
}
