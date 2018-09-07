/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <IterativeRobot.h>
#include <Joystick.h>
#include <Talon.h>
#include <Victor.h>
#include <Jaguar.h>
#include <SD540.h>
#include <DMC60.h>
#include <VictorSP.h>
#include <PWMTalonSRX.h>
#include <PWMVictorSPX.h>
#include <Servo.h>
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
		char joystickVal[10];
		sprintf(joystickVal, "%0.2f", m_stick.GetX());
		char servoVal[10];
		sprintf(servoVal, "%0.2f", (m_stick.GetX()+1.0)/2.0);
		wpi::outs() << "Joystick:  " << joystickVal << ", Motors:  " << joystickVal << ", Servo:  " << servoVal << "\n";
		m_motor1.Set(m_stick.GetX());
		m_motor2.Set(m_stick.GetX());
		m_motor3.Set(m_stick.GetX());
		m_motor4.Set(m_stick.GetX());
		m_motor5.Set(m_stick.GetX());
		m_motor6.Set(m_stick.GetX());
		m_motor7.Set(m_stick.GetX());
		m_motor8.Set(m_stick.GetX());
		m_motor9.Set(m_stick.GetX());
		m_servo1.Set((m_stick.GetX()+1.0)/2.0);
		m_servo2.Set((m_stick.GetX()+1.0)/2.0);

		double servoSpeed1 = m_servo1.GetPosition();
		double servoSpeed2 = m_servo2.GetPosition();
		double motorSpeed1 = m_motor1.GetSpeed();
		double motorSpeed2 = m_motor9.GetSpeed();
		char currServo1Val[10];
		sprintf(currServo1Val, "%0.2f", servoSpeed1);
		char currServo2Val[10];
		sprintf(currServo2Val, "%0.2f", servoSpeed2);
		char currMotor1Val[10];
		sprintf(currMotor1Val, "%0.2f", motorSpeed1);
		char currMotor2Val[10];
		sprintf(currMotor2Val, "%0.2f", motorSpeed2);
		wpi::outs() << "Hardware state:  HCDIO Servo:  " << currServo1Val << ", FlexDIO Servo:  " << currServo2Val <<
				", HCDIO MC:  " << currMotor1Val << ", FlexDIO MC:  " << currMotor2Val << "\n";
		Wait(0.005);
	}

	void TestPeriodic() override {}

private:
	Talon m_motor1{0};
	Talon m_motor1Duplicate{0}; // PWM Motor Conroller already allocated above.
	Victor m_motor2{1};
	Jaguar m_motor3{2};
	SD540 m_motor4{3};
	DMC60 m_motor5{4};
	VictorSP m_motor6{5};
	PWMTalonSRX m_motor7{6};
	PWMVictorSPX m_motor8{7};
	Talon m_motor9{20};
	Servo m_servo1{8};
	Servo m_servo2{21};

	Joystick m_stick{0};
};

START_ROBOT_CLASS(Robot)
