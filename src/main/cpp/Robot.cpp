// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/Timer.h>

using frc::Timer;


void Robot::RobotInit() {
  autoChooser.SetDefaultOption(leftAuto, leftAuto);
  autoChooser.AddOption(middleAuto, middleAuto);
  autoChooser.AddOption(rightAuto, rightAuto);
  frc::SmartDashboard::PutData("Auto Modes", &autoChooser);

  led.SetLength(LedConstants::ledLength);
  led.SetData(ledBuffer);
  led.Start();
  ledColor = LedConstants::ORANGE;

  intakeDirection = 0;
  pullThroughDirection = 0;

  RTelescopingArm.SetControl(Follower{LTelescopingArm.GetDeviceID(), true});
  shooter2.SetControl(Follower{shooter1.GetDeviceID(), false});

  LTelescopingArm.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  RTelescopingArm.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  shooter1.Set(0);
  pullThrough.Set(0);
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
void Robot::DisabledInit() {
  
  LTelescopingArm.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
  RTelescopingArm.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

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
  intakeDirection = 1;
  pullThroughDirection = 1;
  pastPOV = -1;
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  shooter1.Set(-(shooterController.GetR2Axis() + 1) / 2);

  if (!intakeLimitSwitch.Get()) {
    ledColor = LedConstants::YELLOW;
  }
  else if (!shooterLimitSwitch.Get()) {
    ledColor = LedConstants::GREEN;
  }
  else {
    ledColor = LedConstants::ORANGE;
  }
  for (int i = 0; i < LedConstants::ledLength; i++) {
    ledBuffer[i].SetRGB(ledColor.red, ledColor.green, ledColor.blue);
  }
  led.SetData(ledBuffer);

  // if (LTelescopingArm.GetRotorPosition().GetValueAsDouble() < ShooterConstants::telescopingArmMin && shooterController.GetLeftY() > 0) {
  //   LTelescopingArm.Set(0);
  // } else if (LTelescopingArm.GetRotorPosition().GetValueAsDouble() > ShooterConstants::telescopingArmMax && shooterController.GetLeftY() < 0) {
  //   LTelescopingArm.Set(0);
  // } else {
  //   LTelescopingArm.Set(-shooterController.GetLeftY() * 0.2);
  if (driverController.GetPOV() == 0) {
    LTelescopingArm.Set(0.15);
  } else if (driverController.GetPOV() == 180) {
    LTelescopingArm.Set(-0.15);
  } else {
    LTelescopingArm.Set(0);
  }

  shooterPivot.Set(shooterController.GetRightY() * 0.3);
  
  //frc::SmartDashboard::PutNumber("POV Val", shooterController.GetPOV());
 
  if (shooterController.GetPOV() == 0 && pastPOV != 0) {
    intakeDirection = -intakeDirection;
  }
  if (shooterController.GetL1Button()) {
    intakeWheel.Set(intakeDirection * 0.5);
  }
  else {
    intakeWheel.Set(0);
  }
  pastPOV = shooterController.GetPOV();
  
  if (shooterController.GetTriangleButtonPressed()) {
    pullThroughDirection = -pullThroughDirection;
  }
  if (shooterController.GetR1Button()) {
    pullThrough.Set(pullThroughDirection);
  }
  else {
    pullThrough.Set(0);
  }
  // if (shooterDirection != 0 && static_cast<double> (Timer::GetFPGATimestamp() - shooterTime) > 2) {
  //   shooterDirection = 0;
  // }
  //shooterIntake.Set(shooterDirection);

  if (shooterController.GetPSButtonPressed()) {
    pullThroughDirection = 1;
    intakeDirection = 1;
  }
  
  intakePivot.Set(shooterController.GetLeftY() * 0.4);

  // if (!pivotLimitSwitchLower.Get() && shooterController.GetRightY() < 0) {
  //   intakePivot.Set(0);
  // }
  // else if (!pivotLimitSwitchUpper.Get() && shooterController.GetRightY() > 0) {
  //   intakePivot.Set(0);
  // }
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
