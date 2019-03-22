/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/PowerDistributionPanel.h>

using namespace frc;

class Robot : public TimedRobot {
public:
	Robot() {
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() override {
	}

	void TeleopInit() override {}

	void TeleopPeriodic() override {
		wpi::outs() << "PDP Voltage:  " << pdp.GetVoltage() << "\n";
	}

	void TestPeriodic() override {}

private:
	PowerDistributionPanel pdp{7};
};

int main() {
	return frc::StartRobot<Robot>();
}
