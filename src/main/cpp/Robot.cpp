// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include "RobotContainer.h"

using frc::Timer;

// void Robot::intakeNote() {
//   if (pivotLimitSwitchLower.Get() && intakeLimitSwitch.Get()) {
//     intakePivot.Set(.5);
//   }
//   if (!pivotLimitSwitchLower.Get() && intakeLimitSwitch.Get()) {
//     intakePivot.Set(0);
//     intakeWheel.Set(0.5);
//     intakeTime = Timer::GetFPGATimestamp();
//   }
//   if (!intakeLimitSwitch.Get() && !pivotLimitSwitchLower.Get()) {
//     intakeWheel.Set(0);
//     pullThrough.Set(1);
//     if (LTelescopingArm.GetRotorPosition().GetValueAsDouble() <
//     ShooterConstants::telescopingArmMax) {
//       LTelescopingArm.Set(0.3);
//     }
//     else {
//       LTelescopingArm.Set(0);
//     }
//     if (pivotLimitSwitchUpper.Get()) {
//       intakePivot.Set(0.5);
//     }
//   }
//   if (!intakeLimitSwitch.Get() && !pivotLimitSwitchUpper.Get()) {
//     if (LTelescopingArm.GetRotorPosition.GetValueAsDouble() >
//     ShooterConstants::telescopingArmMin) {
//       LTelescopingArm.Set(-0.3);
//     }
//     else {
//       LTelescopingArm.Set(0);
//     }
//   }
//   if (static_cast<double>(Timer::GetFPGATimestamp() - intakeTime) > 10 &&
//       intakeLimitSwitch.Get()) {
//     intakeWheel.Set(0);
//     intakePivot.Set(-0.5);
//   }
// }

void Robot::RobotInit() {
  autoChooser.SetDefaultOption(position1, position1);
  autoChooser.AddOption(position2, position2);
  frc::SmartDashboard::PutData("Auto Modes", &autoChooser);

  allianceChooser.SetDefaultOption(redAlliance, redAlliance);
  allianceChooser.AddOption(blueAlliance, blueAlliance);
  frc::SmartDashboard::PutData("Alliance Selector", &allianceChooser);

  led.SetLength(LedConstants::ledLength);
  led.SetData(ledBuffer);
  led.Start();
  ledColor = LedConstants::ORANGE;

  RTelescopingArm.SetControl(Follower{LTelescopingArm.GetDeviceID(), true});
  shooter2.SetControl(Follower{shooter1.GetDeviceID(), false});

  LTelescopingArm.SetNeutralMode(
      ctre::phoenix6::signals::NeutralModeValue::Brake);
  RTelescopingArm.SetNeutralMode(
      ctre::phoenix6::signals::NeutralModeValue::Brake);

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
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  LTelescopingArm.SetNeutralMode(
      ctre::phoenix6::signals::NeutralModeValue::Coast);
  RTelescopingArm.SetNeutralMode(
      ctre::phoenix6::signals::NeutralModeValue::Coast);
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  int multiplier = 1;
  noteAutoLoaderAutomation.state = off;
  std::string selection = autoChooser.GetSelected();
  // if (autoChooser.GetSelected() == leftAuto) {
  //   multiplier = -1;
  // }
  autoTime = Timer::GetFPGATimestamp();
  currentAuto.state = moveToShooter;
  autonomousCommand1 = container.GetAutonomousCommand1(
      autoChooser.GetSelected(), allianceChooser.GetSelected());
  autonomousCommand2 =
      container.GetAutonomousCommand2(allianceChooser.GetSelected());
  autonomousCommand3 =
      container.GetAutonomousCommand2(allianceChooser.GetSelected());
  autoTime = Timer::GetFPGATimestamp();
}

void Robot::AutonomousPeriodic() {
  double timeDiff = static_cast<double>(Timer::GetFPGATimestamp() - autoTime);
  if (currentAuto.stateChange) {
    currentAuto.stateChange = false;
  } else if (timeDiff > 12) {
    currentAuto.state = taxi;
    currentAuto.stateChange = true;
  } else if (timeDiff > 11) {
    currentAuto.state = shootNote2;
    currentAuto.stateChange = true;
  } else if (timeDiff > 9) {
    currentAuto.state = moveBackToShooter;
    currentAuto.stateChange = true;
  } else if (timeDiff > 5) {
    currentAuto.state = intakingNoteAutonomous;
    currentAuto.stateChange = true;
  } else if (timeDiff > 3) {
    currentAuto.state = moveToNote;
    currentAuto.stateChange = true;
  } else if (timeDiff > 2) {
    currentAuto.state = shootNote1;
    currentAuto.stateChange = true;
  }

  if (currentAuto.stateChange) {
    if (currentAuto.state == moveToShooter) {
      autonomousCommand1->Schedule();
    } else if (currentAuto.state == shootNote1) {
      autonomousCommand1->Cancel();
      shooter1.Set(-1);
      pullThrough.Set(0.5);
    } else if (currentAuto.state == moveToNote) {
      autonomousCommand2->Schedule();
      shooter1.Set(0);
      pullThrough.Set(0);
    } else if (currentAuto.state == intakingNoteAutonomous) {
      autonomousCommand2->Cancel();
      // intake code goes here
    } else if (currentAuto.state == moveBackToShooter) {
      autonomousCommand3->Schedule();
    } else if (currentAuto.state == shootNote2) {
      autonomousCommand3->Schedule();
      shooter1.Set(-1);
      pullThrough.Set(0.5);
    } else {
      autonomousCommand3->Cancel();
      shooter1.Set(0);
      pullThrough.Set(0);
    }
  }
}

void Robot::TeleopInit() {
  noteAutoLoaderAutomation.state = off;
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (currentAutonomousCommand != nullptr) {
    currentAutonomousCommand->Cancel();
    currentAutonomousCommand = nullptr;
  }
  shooter1.Set(0);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  if (shooterController.GetPOV() == 0 &&
      noteAutoLoaderAutomation.state == off) {
    noteAutoLoaderAutomation.state = intakeDeploying;
  }

  if (noteAutoLoaderAutomation.state != off) {
    automateNoteLoading();
  }

  shooter1.Set(-(shooterController.GetR2Axis() + 1) / 2);

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

  if (driverController.GetPOV() == 0 &&
      LTelescopingArm.GetRotorPosition().GetValueAsDouble() <
          ShooterConstants::telescopingArmMax) {
    LTelescopingArm.Set(0.15);
  } else if (driverController.GetPOV() == 180 &&
             LTelescopingArm.GetRotorPosition().GetValueAsDouble() >
                 ShooterConstants::telescopingArmMin) {
    LTelescopingArm.Set(-0.15);
  } else {
    LTelescopingArm.Set(0);
  }

  shooterPivot.Set(shooterController.GetLeftY() * 0.3);

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

  if (shooterController.GetSquareButton()) {
    shooter1.Set(0.25);
  }

  // intakePivot.Set(shooterController.GetRightY() * 0.4);
}

void Robot::automateNoteLoading() {
  if (shooterController.GetPSButton()) {
    noteAutoLoaderAutomation.state = off;
    return;
  }
  if (noteAutoLoaderAutomation.state == intakeDeploying) {
    if (pivotLimitSwitchLower.Get()) {
      noteAutoLoaderAutomation.state = intakingNote;
      noteAutoLoaderAutomation.intakeDeployStartTime =
          Timer::GetFPGATimestamp();
      intakePivot.Set(0);
      return;
    }
    intakePivot.Set(0.3);
  } else if (noteAutoLoaderAutomation.state == intakingNote) {
    if (intakeLimitSwitch.Get()) {
      noteAutoLoaderAutomation.state = intakeRetracting;
      intakeWheel.Set(0);
      return;
    } else if (static_cast<double>(Timer::GetFPGATimestamp() -
                   noteAutoLoaderAutomation.intakeDeployStartTime) >
               10) {
      noteAutoLoaderAutomation.state == cancelling;
      intakeWheel.Set(0);
      return;
    }
    intakeWheel.Set(0.5);
  } else if (noteAutoLoaderAutomation.state == intakeRetracting) {
    if (pivotLimitSwitchUpper.Get()) {
      noteAutoLoaderAutomation.state = shooterDeploying;
      intakePivot.Set(0);
      return;
    }
    intakePivot.Set(-0.3);
  } else if (noteAutoLoaderAutomation.state == shooterDeploying) {
    if (shooterLimitSwitch.Get() &&
        LTelescopingArm.GetRotorPosition().GetValueAsDouble() == 0.0) {
      noteAutoLoaderAutomation.state == noteLoading;
      shooterPivot.Set(0);
      LTelescopingArm.Set(0);
      return;
    }
    if (!shooterLimitSwitch.Get()) {
      shooterPivot.Set(0.3);
    } else {
      shooterPivot.Set(0);
    }
    if (LTelescopingArm.GetRotorPosition().GetValueAsDouble() > 0.0) {
      LTelescopingArm.Set(0.3);
    } else {
      LTelescopingArm.Set(0);
    }
  } else if (noteAutoLoaderAutomation.state == noteLoading) {
    if (shooterLoadedLimitSwitch.Get()) {
      noteAutoLoaderAutomation.state = noteLoaded;
      pullThrough.Set(0);
    }
    pullThrough.Set(1);
  } else if (noteAutoLoaderAutomation.state == noteLoaded) {
    if (!shooterLoadedLimitSwitch.Get()) {
      noteAutoLoaderAutomation.state = off;
    }
  } else if (noteAutoLoaderAutomation.state == cancelling) {
    if (pivotLimitSwitchUpper.Get()) {
      noteAutoLoaderAutomation.state = off;
      intakePivot.Set(0);
      return;
    }
    intakePivot.Set(-0.3);
  }
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
