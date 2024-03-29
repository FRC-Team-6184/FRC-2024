// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/PS5Controller.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using frc::PS5Controller;
using frc::SendableChooser;
using frc::XboxController;
using frc2::Command;

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  Command* GetAutonomousCommand1(double xDir, double yDir, double rotation);
  Command* GetAutonomousCommand2(double xDir, double yDir, double rotation);
  Command* GetAutonomousCommand3(double xDir, double yDir, double rotation);
  Command* GetAutonomousCommand4(double xDir, double yDir, double rotation);

  XboxController driverController{OIConstants::driverControllerPort};

  // bool autonomousShoot;

  // void setAutonomousShoot(bool autonomousShout);

  // The driver's controller
 private:
  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem driveSubsystem;

  void ConfigureButtonBindings();
};
