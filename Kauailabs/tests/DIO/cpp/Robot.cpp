/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <IterativeRobot.h>
#include <Joystick.h>
#include <DigitalInput.h>
#include <DigitalOutput.h>
#include <Timer.h>

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

	void TeleopInit() override {}

	void TeleopPeriodic() override {
		if (m_stick.GetRawButton(0)) {
			wpi::outs() << "Pressed\n";
			m_output1.Set(1);
			m_output2.Set(1);
		} else {
			wpi::outs() << "Released\n";
			m_output1.Set(0);
			m_output2.Set(0);
		}
		wpi::outs() << "Input 1:  " << m_input1.Get() << ", Input 2:  " << m_input2.Get() << "\n";		
		Wait(0.005);
	}

	void TestPeriodic() override {}

private:
	DigitalInput m_input1{0};
	DigitalInput m_input2{1};
	DigitalOutput m_output1{12};
	DigitalOutput m_output2{13};

	Joystick m_stick{0};
};

START_ROBOT_CLASS(Robot)
