package com.kauailabs.vmx.wpilib.Examples.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Robot extends TimedRobot {
	WPI_TalonSRX talonSRX;
	int count;
	public Robot() {
		talonSRX = new WPI_TalonSRX(2);
		count = 0;
	}

	public void robotInit() {
		System.out.println("WPI Library Java TalonSRX Example Program");
	}

	public void teleopPeriodic() {
		talonSRX.feed();

		boolean is_alive = talonSRX.isAlive();

		double new_talon_srx_output = count % 100;
		new_talon_srx_output /= 100;

		double bus_voltage = talonSRX.getBusVoltage();
		double temp = talonSRX.getTemperature();
		String srx_name = talonSRX.getName();

		talonSRX.set(new_talon_srx_output);

		double talon_srx_output = talonSRX.get();

		System.out.println("TalonSRX " + srx_name + " " + 
			(is_alive ? "Alive" : "Not Alive") + 
			" - Output:  " + talon_srx_output + 
			" - Bus Voltage:  " + bus_voltage + 
			" - Temperature:  " + temp);
	}

	public void disabled() {
	}
}
