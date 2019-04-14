package com.kauailabs.vmx.wpilib.Examples.PCM;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;

public class Robot extends TimedRobot {

	DoubleSolenoid solenoid;
	Compressor compressor;
	int count;

	public Robot() {
		solenoid = new DoubleSolenoid(1,0,1);
		compressor = new Compressor(1);
		count = 0;
	}

	public void robotInit() {
		System.out.println("WPI Library Java PCM Example Program");
	}

	public void teleopPeriodic() {
		int step = count % 100;
		if (step == 0) {
			solenoid.set(DoubleSolenoid.Value.kForward);
		} else if (step == 50){
			solenoid.set(DoubleSolenoid.Value.kReverse);
		}

		int blacklist = solenoid.getPCMSolenoidBlackList();
		boolean voltage_sticky_fault = solenoid.getPCMSolenoidVoltageStickyFault();
		boolean voltage_fault = solenoid.getPCMSolenoidVoltageFault();

		System.out.println("Double Solenoid Value:  " + solenoid.get());
		if (blacklist != 0) System.out.println("Solenoid blacklist:  " + blacklist);
		if (voltage_fault) System.out.println("Solenoid voltage fault!");
		if (voltage_sticky_fault) System.out.println("Solenoid voltage sticky fault!");
		count++;		
	}

	public void disabled() {
	}
}
