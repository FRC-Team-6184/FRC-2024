// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/CommandScheduler.h>
#include <wpi/fs.h>

#include "RobotContainer.h"

using frc::Shuffleboard;
using frc::ShuffleboardTab;
using frc::SmartDashboard;
using frc::Timer;

void Robot::RobotInit() {
  led.SetLength(LedConstants::ledLength);
  led.SetData(ledBuffer);
  led.Start();
  ledColor = LedConstants::ORANGE;

  RTelescopingArm.SetControl(Follower{LTelescopingArm.GetDeviceID(), true});
  shooter2.SetControl(Follower{shooter1.GetDeviceID(), false});

  LTelescopingArm.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  RTelescopingArm.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

  noteAutoLoaderAutomation.telescopingArmOffset = LTelescopingArm.GetRotorPosition().GetValueAsDouble();

  shooter1.Set(0);
  pullThrough.Set(0);

  noteAutoLoaderAutomation.state = off;

  initializeShuffleBoard();
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
  populateShuffleBoard();
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
  int alliance = 1;
  noteAutoLoaderAutomation.state = off;

  autoTime = Timer::GetFPGATimestamp();

  if (allianceChooser.GetSelected() == "Red Alliance") {
    alliance = 1;
  } else {
    alliance = -1;
  }

  if (autoChooser.GetSelected() == position1) {
    currentAuto.twoNote = false;
    autonomousCommand1 = container.SideTaxi1(alliance);
    autonomousCommand2 = container.SideTaxi1Part2(alliance);
  } else if (autoChooser.GetSelected() == position2) {
    currentAuto.twoNote = true;
    autonomousCommand1 = container.MiddleTaxi(alliance);
    autonomousCommand2 = container.MiddleTaxiPart2(alliance);
  } else {
    currentAuto.twoNote = false;
    autonomousCommand1 = container.SideTaxi2(alliance);
    autonomousCommand2 = container.SideTaxi2Part2(alliance);
  }

  currentAuto.state = autoOff;
  currentAuto.lastTickState = autoOff;
}

void Robot::AutonomousPeriodic() {
  double timeDiff = static_cast<double>(Timer::GetFPGATimestamp() - autoTime);

  if (currentAuto.twoNote) {
    if (timeDiff > 8) {
      currentAuto.state = autoOff;
    } else if (timeDiff > 3) {
      currentAuto.state = runningAuto2;
    } else if (timeDiff > 2) {
      currentAuto.state = runningAuto1;
    } else {
      currentAuto.state = shootingNote;
    }
  } else {
    if (timeDiff > 11) {
      currentAuto.state = autoOff;
    } else if (timeDiff > 6) {
      currentAuto.state = runningAuto2;
    } else if (timeDiff > 2) {
      currentAuto.state = runningAuto1;
    } else {
      currentAuto.state = shootingNote;
    }
  }

  if (currentAuto.state == shootingNote) {
    shooter1.Set(1);
    pullThrough.Set(1);
  } else {
    shooter1.Set(0);
    pullThrough.Set(0);
  }

  if (currentAuto.state != currentAuto.lastTickState) {
    if (currentAuto.lastTickState == runningAuto1) {
      autonomousCommand1->Cancel();
    } else if (currentAuto.lastTickState == runningAuto2) {
      autonomousCommand2->Cancel();
    } else if (currentAuto.lastTickState == runningAuto3) {
      autonomousCommand3->Cancel();
    } else if (currentAuto.lastTickState == runningAuto4) {
      autonomousCommand4->Cancel();
    }
  }

  if (currentAuto.state == runningAuto1) {
    autonomousCommand1->Schedule();
  } else if (currentAuto.state == runningAuto2) {
    autonomousCommand2->Schedule();
  } else if (currentAuto.state == runningAuto3) {
    autonomousCommand3->Schedule();
  } else if (currentAuto.state == runningAuto4) {
    autonomousCommand4->Schedule();
  }
  currentAuto.lastTickState = currentAuto.state;
}

void Robot::TeleopInit() {
  noteAutoLoaderAutomation.state = off;
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (currentAuto.lastTickState == runningAuto1) {
    autonomousCommand1->Cancel();
  } else if (currentAuto.lastTickState == runningAuto2) {
    autonomousCommand2->Cancel();
  } else if (currentAuto.lastTickState == runningAuto3) {
    autonomousCommand3->Cancel();
  } else if (currentAuto.lastTickState == runningAuto4) {
    autonomousCommand4->Cancel();
  }

  shooter1.Set(0);
  shooterDir = 0;
  shooterIntakingNote = false;
  intakPivotOverride = false;
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  intakePivot.Set(0);
  intakeWheel.Set(0);

  if (shooterController.GetPOV() == 180 && noteAutoLoaderAutomation.state == off) {
    noteAutoLoaderAutomation.state = intakeDeploying;
  }

  shooter1.Set(-(shooterController.GetR2Axis() + 1) / 2);
  pullThrough.Set((shooterController.GetR2Axis() + 1) / 2);
  intakeWheel.Set(-(shooterController.GetR2Axis() + 1) / 2);

  if (noteAutoLoaderAutomation.state != off) {
    automateNoteLoading();
  }

  if (!intakeLimitSwitch.Get()) {
    ledColor = LedConstants::YELLOW;
  } else if (!shooterLimitSwitch.Get()) {
    ledColor = LedConstants::GREEN;
  } else {
    ledColor = LedConstants::ORANGE;
  }
  for (int i = 0; i < LedConstants::ledLength; i++) {
    ledBuffer[i].SetRGB(ledColor.red, ledColor.green, ledColor.blue);
  }
  led.SetData(ledBuffer);

  if (shooterController.GetTriangleButton()) {
    intakeWheel.Set(-1);
  }

  if (shooterController.GetLeftY() > 0.1 && pivotLimitSwitchUpper.Get()) {
    intakePivot.Set(shooterController.GetLeftY() * 0.2);
  } else if (shooterController.GetLeftY() < -0.1 && pivotLimitSwitchLower.Get()) {
    intakePivot.Set(shooterController.GetLeftY() * 0.2);
  }

  // if (shooterDir != 0) {
  //   shooter1.Set(shooterDir * 0.5);
  //   pullThrough.Set(shooterDir * 0.5);
  // } else {
  //   pullThrough.Set(0);
  // }

  // if (driverController.GetPOV() == 0 &&
  //     LTelescopingArm.GetRotorPosition().GetValueAsDouble() <
  //         ShooterConstants::telescopingArmMax) {
  //   LTelescopingArm.Set(0.15);
  // } else if (driverController.GetPOV() == 180 &&
  //            LTelescopingArm.GetRotorPosition().GetValueAsDouble() >
  //                ShooterConstants::telescopingArmMin) {
  //   LTelescopingArm.Set(-0.15);
  // } else {
  //   LTelescopingArm.Set(0);
  // }
  if (shooterController.GetLeftY() > 0 && !shooterLimitSwitch.Get()) {
    shooterPivot.Set(0);
  } else {
    shooterPivot.Set(shooterController.GetLeftY() * 0.3);
  }

  // if (shooterController.GetPOV() == 270) {
  //   intakeDirection = 1;
  //   pullThroughDirection = 0;
  // } else if (shooterController.GetPOV() == 0) {
  //   intakeDirection = 0;
  //   pullThroughDirection = 1;
  // } else if (shooterController.GetPOV() == 90) {
  //   intakeDirection = -1;
  //   pullThroughDirection = 1;
  // } else {
  //   intakeDirection = 0;
  //   pullThroughDirection = 0;
  // }

  // if (shooterController.GetCircleButton()) {
  //   intakeDirection = -1;
  //   pullThroughDirection = 0;
  // }

  // intakeWheel.Set(intakeDirection * 0.5);
  // pullThrough.Set(pullThroughDirection);

  if (shooterController.GetPOV() == 0) {
    shooterIntakingNote = true;
  }
  if (shooterIntakingNote && !shooterLoadedLimitSwitch.Get()) {
    shooterIntakingNote = false;
  } else if (shooterIntakingNote) {
    shooter1.Set(0.25);
    pullThrough.Set(-0.25);
  }

  // intakePivot.Set(shooterController.GetRightY() * 0.4);
}

void Robot::automateNoteLoading() {
  if (shooterController.GetPSButton()) {
    noteAutoLoaderAutomation.state = off;
    return;
  }
  if (noteAutoLoaderAutomation.state == intakeDeploying) {
    if (!pivotLimitSwitchLower.Get()) {
      noteAutoLoaderAutomation.state = intakingNote;
      noteAutoLoaderAutomation.intakeDeployStartTime = Timer::GetFPGATimestamp();
      intakePivot.Set(0);
      return;
    }
    intakeWheel.Set(1);
    intakePivot.Set(0.15);
  } else if (noteAutoLoaderAutomation.state == intakingNote) {
    if (!intakeLimitSwitch.Get() || shooterController.GetPOV() == 270) {
      noteAutoLoaderAutomation.state = intakeRetracting;
      intakeWheel.Set(0);
      return;
    } else if (static_cast<double>(Timer::GetFPGATimestamp() - noteAutoLoaderAutomation.intakeDeployStartTime) > 10) {
      noteAutoLoaderAutomation.state == cancelling;
      return;
    }
    intakeWheel.Set(1.0);
    pullThrough.Set(1.0);
    intakePivot.Set(0.0);
  } else if (noteAutoLoaderAutomation.state == intakeRetracting) {
    if (!pivotLimitSwitchUpper.Get()) {
      noteAutoLoaderAutomation.state = noteLoading;
      intakePivot.Set(0);
      return;
    }
    intakePivot.Set(-0.2);
    pullThrough.Set(1.0);
  } else if (noteAutoLoaderAutomation.state == noteLoading) {
    if (!shooterLoadedLimitSwitch.Get()) {
      noteAutoLoaderAutomation.state = noteLoaded;
      pullThrough.Set(0);
      intakeWheel.Set(0);
    }
    pullThrough.Set(1.0);
    intakeWheel.Set(-1.0);
  } else if (noteAutoLoaderAutomation.state == noteLoaded) {
    if (shooterLoadedLimitSwitch.Get()) {
      noteAutoLoaderAutomation.state = off;
    }
  } else if (noteAutoLoaderAutomation.state == cancelling) {
    if (pivotLimitSwitchUpper.Get()) {
      noteAutoLoaderAutomation.state = off;
      intakePivot.Set(0);
      intakeWheel.Set(0);
      pullThrough.Set(0);
      LTelescopingArm.Set(0);
      return;
    }
    intakePivot.Set(-0.1);
  }
}

std::string Robot::noteAutoLoaderStateString() {
  if (noteAutoLoaderAutomation.state == off) {
    return "Off";
  } else if (noteAutoLoaderAutomation.state == intakeDeploying) {
    return "Intake Deploying";
  } else if (noteAutoLoaderAutomation.state == intakingNote) {
    return "Intaking Note";
  } else if (noteAutoLoaderAutomation.state == intakeRetracting) {
    return "Intake Retracting";
  } else if (noteAutoLoaderAutomation.state == noteLoading) {
    return "Note Loading";
  } else if (noteAutoLoaderAutomation.state == noteLoaded) {
    return "Note Loaded";
  } else {
    return "Cancelling";
  }
}

void Robot::initializeShuffleBoard() {
  autoChooser.SetDefaultOption(position1, position1);
  autoChooser.AddOption(position2, position2);
  autoChooser.AddOption(position3, position3);
  SmartDashboard::PutData("Auto Modes", &autoChooser);

  allianceChooser.SetDefaultOption(redAlliance, redAlliance);
  allianceChooser.AddOption(blueAlliance, blueAlliance);
  SmartDashboard::PutData("Alliance Selector", &allianceChooser);

  ShuffleboardTab& tab = Shuffleboard::GetTab("Sensors");
}

void Robot::populateShuffleBoard() {
  SmartDashboard::PutBoolean("Shooter Intaking Note", shooterIntakingNote);

  SmartDashboard::PutString("Intake State", Robot::noteAutoLoaderStateString());
  SmartDashboard::PutNumber("Intake Progress Bar", (int)noteAutoLoaderAutomation.state / 6);
  SmartDashboard::PutBoolean("Running Auto 1", currentAuto.state == runningAuto1);
  SmartDashboard::PutBoolean("Running Auto 2", currentAuto.state == runningAuto2);
  SmartDashboard::PutBoolean("Running Auto 3", currentAuto.state == runningAuto3);
  SmartDashboard::PutBoolean("Running Auto 4", currentAuto.state == runningAuto4);
  SmartDashboard::PutBoolean("Shooting Note", currentAuto.state == shootingNote);
  SmartDashboard::PutBoolean("Intaking Note Autonomous", currentAuto.state == intakingNoteAutonomous);

  SmartDashboard::PutBoolean("Pivot Switch Lower", pivotLimitSwitchLower.Get());
  SmartDashboard::PutBoolean("Pivot Switch Upper", pivotLimitSwitchUpper.Get());
  SmartDashboard::PutBoolean("Shooter Loaded Limit Switch", shooterLoadedLimitSwitch.Get());
  SmartDashboard::PutBoolean("Intake Note Limit Switch", intakeLimitSwitch.Get());
  SmartDashboard::PutBoolean("Shooter Head", shooterLimitSwitch.Get());

  SmartDashboard::PutNumber("Intake Pivot", intakePivot.Get());
  SmartDashboard::PutNumber("Intake Wheel", intakeWheel.Get());
  SmartDashboard::PutNumber("Pull Through", pullThrough.Get());
  SmartDashboard::PutNumber("Left Telescoping Arm Position", LTelescopingArm.Get());

  SmartDashboard::PutString("Left Telescoping Arm Position", LTelescopingArm.GetPosition().ToString());
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
