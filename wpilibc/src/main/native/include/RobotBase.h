/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <thread>

#include <HAL/HAL.h>
#include <wpi/raw_ostream.h>

#include "Base.h"


namespace frc {

class DriverStation;

template <class Robot>
int StartRobot() {
  if (!HAL_Initialize(500, 0)) {
    wpi::errs() << "FATAL ERROR: HAL could not be initialized\n";
    return -1;
  }
  HAL_Report(HALUsageReporting::kResourceType_Language,
             HALUsageReporting::kLanguage_CPlusPlus);
  wpi::outs() << "\n********** Robot program starting **********\n";
  static Robot robot;
  robot.StartCompetition();

  return 0;
}

#define START_ROBOT_CLASS(_ClassName_)                                 \
  WPI_DEPRECATED("Call frc::StartRobot<" #_ClassName_                  \
                 ">() in your own main() instead of using the "        \
                 "START_ROBOT_CLASS(" #_ClassName_ ") macro.")         \
  int StartRobotClassImpl() { return frc::StartRobot<_ClassName_>(); } \
  int main() { return StartRobotClassImpl(); }

/**
 * Implement a Robot Program framework.
 *
 * The RobotBase class is intended to be subclassed by a user creating a robot
 * program. Overridden Autonomous() and OperatorControl() methods are called at
 * the appropriate time as the match proceeds. In the current implementation,
 * the Autonomous code will run to completion before the OperatorControl code
 * could start. In the future the Autonomous code might be spawned as a task,
 * then killed at the end of the Autonomous period.
 */
class RobotBase {
 public:
  bool IsEnabled() const;
  bool IsDisabled() const;
  bool IsAutonomous() const;
  bool IsOperatorControl() const;
  bool IsTest() const;
  bool IsNewDataAvailable() const;
  static std::thread::id GetThreadId();
  virtual void StartCompetition() = 0;

  static constexpr bool IsReal() {
#ifdef __FRC_ROBORIO__
    return true;
#else
    return false;
#endif
  }

  static constexpr bool IsSimulation() { return !IsReal(); }

 protected:
  RobotBase();
  virtual ~RobotBase() = default;

  RobotBase(const RobotBase&) = delete;
  RobotBase& operator=(const RobotBase&) = delete;

  DriverStation& m_ds;

  static std::thread::id m_threadId;
};

}  // namespace frc
