/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/IterativeRobot.h>
#include <frc/Timer.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

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
	}

	void TeleopPeriodic() override {

		talonSRX.Feed();

		bool is_alive = talonSRX.IsAlive();

		double new_talon_srx_output = count % 100;
		new_talon_srx_output /= 100;

		double bus_voltage = talonSRX.GetBusVoltage();
		double temp = talonSRX.GetTemperature();
		std::string srx_name = talonSRX.GetName();

		talonSRX.Set(new_talon_srx_output);

		double talon_srx_output = talonSRX.Get();

		wpi::outs() << "TalonSRX " << srx_name << " " << (is_alive ? "Alive" : "Not Alive") << " - Output:  " << talon_srx_output << " - Bus Voltage:  " << bus_voltage << " - Temperature:  " << temp << "\n";

		count++;

		Wait(0.005);
	}

	void TestPeriodic() override {}

private:
	WPI_TalonSRX talonSRX{2};
	int count{0};
};

START_ROBOT_CLASS(Robot)
