/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/IterativeRobot.h>
#include <frc/Timer.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>

using namespace frc;

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
		compressor.ClearAllPCMStickyFaults();
	}

	void TeleopPeriodic() override {
		int step = count % 100;
		if (step == 0) {
			solenoid.Set(DoubleSolenoid::Value::kForward);
		} else if (step == 50){
			solenoid.Set(DoubleSolenoid::Value::kReverse);
		}
		int blacklist = solenoid.GetPCMSolenoidBlackList();
		bool voltage_sticky_fault = solenoid.GetPCMSolenoidVoltageStickyFault();
		bool voltage_fault = solenoid.GetPCMSolenoidVoltageFault();


		wpi::outs() << "Double Solenoid Value:  " << solenoid.Get() << "\n";
		if (blacklist) wpi::outs() << "Solenoid blacklist:  " << blacklist << "\n";
		if (voltage_fault) wpi::outs() << "Solenoid voltage fault!" << "\n";
		if (voltage_sticky_fault) wpi::outs() << "Solenoid voltage sticky fault!" << "\n";
		count++;
		Wait(0.005);
	}

	void TestPeriodic() override {}

private:
	DoubleSolenoid solenoid{1,0,1};
	Compressor compressor{1};
	int count{0};
};

START_ROBOT_CLASS(Robot)
