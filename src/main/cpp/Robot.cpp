// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>


void Robot::RobotInit() {
  autoChooser.SetDefaultOption(leftAuto, leftAuto);
  autoChooser.AddOption(middleAuto, middleAuto);
  autoChooser.AddOption(rightAuto, rightAuto);
  frc::SmartDashboard::PutData("Auto Modes", &autoChooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  int multiplier = 1;
  std::string selection = autoChooser.GetSelected();
  if (autoChooser.GetSelected() == leftAuto) {
    multiplier = -1;
  }
  autonomousCommand = container.GetAutonomousCommand(autoChooser.GetSelected());

  if (autonomousCommand != nullptr) {
    autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (autonomousCommand != nullptr) {
    autonomousCommand->Cancel();
    autonomousCommand = nullptr;
  }
  shooterOn = false;
  intakeWheelOn = false;
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  shooter2.SetControl(Follower{shooter1.GetDeviceID(), false});
  if (shooterController.GetCircleButtonPressed()) {
    shooterOn = !shooterOn;
  }
  if (shooterOn) {
    shooter1.Set(-1);
  } else {
    shooter1.Set(0);
  }

  if (shooterController.GetSquareButtonPressed()) {
    intakeWheelOn = !intakeWheelOn;
  }
   if (intakeWheelOn) {
    intakeWheel.Set(1);
  } else {
    intakeWheel.Set(0);
  }

}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
