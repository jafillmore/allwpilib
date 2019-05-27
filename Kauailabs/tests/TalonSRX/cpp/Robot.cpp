/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/IterativeRobot.h>
#include <frc/Timer.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Robot : public IterativeRobot {
public:
	Robot() {
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() override {
		Wait(0.02);
	}

	void TeleopInit() override {
		talonSRX.ClearStickyFaults();
	}

	void TeleopPeriodic() override {

		talonSRX.Feed();
		c_FeedEnable(100);	// Needed to enable the motors for 100ms (in systems w/out the FRC reference implementation).

		ctre::phoenix::ErrorCode errcode = talonSRX.GetLastError();
		std::string srx_name = talonSRX.GetName();
		if (errcode != ctre::phoenix::ErrorCode::OK) {
			wpi::outs() << "TalonSRX " << srx_name << " Error Code:  " << errcode << "\n";
		}

		bool is_alive = talonSRX.IsAlive();

		double new_talon_srx_output = count % 100;
		new_talon_srx_output /= 100;
		new_talon_srx_output *= 2;
		new_talon_srx_output -= 1;

		double bus_voltage = talonSRX.GetBusVoltage();
		double temp = talonSRX.GetTemperature();
		int firmware_version = talonSRX.GetFirmwareVersion();
		double motor_output_percent = talonSRX.GetMotorOutputPercent();
		double motor_output_voltage = talonSRX.GetMotorOutputVoltage();

		/**
		 * @return applied voltage to motor  in volts.
		 */
		talonSRX.Set(new_talon_srx_output);

		double talon_srx_output = talonSRX.Get();

		wpi::outs() << "TalonSRX " << srx_name <<
				" " << (is_alive ? "Alive" : "Not Alive") <<
				"Firmware:  " << firmware_version <<
				" - Output:  " << talon_srx_output <<
				" - Requested Output:  " << new_talon_srx_output <<
				" - Bus Voltage:  " << bus_voltage <<
				" - Motor Output V:  " << motor_output_voltage <<
				" - Motor Output %:  " << motor_output_percent <<
				" - Temperature:  " << temp << "\n";

		count++;

		Wait(0.005);
	}

	void TestPeriodic() override {}

private:
	WPI_TalonSRX talonSRX{2};
	int count{0};
};

START_ROBOT_CLASS(Robot)
