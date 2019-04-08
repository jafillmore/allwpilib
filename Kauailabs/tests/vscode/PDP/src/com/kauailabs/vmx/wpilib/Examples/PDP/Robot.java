package com.kauailabs.vmx.wpilib.Examples.PDP;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class Robot extends TimedRobot {
	PowerDistributionPanel pdp;
	public Robot() {
		pdp = new PowerDistributionPanel(7);
	}

	public void robotInit() {
		System.out.println("WPI Library Java PDP Example Program");
	}

	public void teleopPeriodic() {
		System.out.println("PDP Voltage:  " + pdp.getVoltage());
	}

	public void disabled() {
	}
}
