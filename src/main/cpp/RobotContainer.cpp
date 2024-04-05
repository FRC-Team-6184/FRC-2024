// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "Robot.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;
using frc::ApplyDeadband;
using frc::PIDController;
using frc::Pose2d;
using frc::ProfiledPIDController;
using frc::TrajectoryConfig;
using frc::TrajectoryGenerator;
using frc::Translation2d;
using frc::XboxController;
using frc2::Command;
using frc2::InstantCommand;
using frc2::JoystickButton;
using frc2::RunCommand;
using frc2::SequentialCommandGroup;
using frc2::SwerveControllerCommand;
using std::move;
using std::string;
using std::numbers::pi;
using units::meters_per_second_t;
using units::radian_t;
using units::radians;
using units::radians_per_second_t;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  driveSubsystem.SetDefaultCommand(RunCommand(
      [this] {
        double speedMultiplier = speedMode::NORMAL_SPEED;
        if (driverController.GetLeftBumper() && driverController.GetRightBumper()) {
          speedMultiplier = speedMode::LUDICROUS_SPEED;
        } else if (driverController.GetLeftBumper()) {
          speedMultiplier = speedMode::TURBO_SPEED;
        } else if (driverController.GetRightBumper()) {
          speedMultiplier = speedMode::TURTLE_SPEED;
        }

        double dirAngle = atan2(driverController.GetLeftY(), driverController.GetLeftX()) + startAngle;
        double dirMag = sqrt(pow(driverController.GetLeftY(), 2) + pow(driverController.GetLeftX(), 2));
        double yDir = sin(dirAngle);
        double xDir = cos(dirAngle);

        driveSubsystem.Drive(
            meters_per_second_t{ApplyDeadband(-yDir * dirMag * speedMultiplier, OIConstants::driveDeadband)},
            meters_per_second_t{ApplyDeadband(-xDir * dirMag * speedMultiplier, OIConstants::driveDeadband)},
            radians_per_second_t{
                ApplyDeadband(-driverController.GetRightX() * speedMultiplier, OIConstants::driveDeadband)},
            true, true);
      },
      {&driveSubsystem}));
}

void RobotContainer::SetStartAngle(double angle) { startAngle = angle; }

void RobotContainer::ConfigureButtonBindings() {
  // JoystickButton(&driverController, XboxController::Button::kRightBumper)
  //.WhileTrue(new RunCommand([this] { driveSubsystem.SetX(); },
  //{&driveSubsystem}));
}

Command* RobotContainer::SideTaxi1(int alliance) {
  return new SequentialCommandGroup(
      InstantCommand([this]() { driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
}
Command* RobotContainer::SideTaxi1Part2(int alliance) {
  if (alliance == 1) {
    return new SequentialCommandGroup(
        InstantCommand([this]() { driveSubsystem.Drive(0.25_mps, -0.433_mps, 0_rad_per_s, true, true); }, {}));
  } else {
    return new SequentialCommandGroup(
        InstantCommand([this]() { driveSubsystem.Drive(0.25_mps, 0.433_mps, 0_rad_per_s, true, true); }, {}));
  }
}
Command* RobotContainer::GoToNote(int alliance) {
  return new SequentialCommandGroup(
      InstantCommand([this]() { driveSubsystem.Drive(1_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
}
Command* RobotContainer::ReturnToSpeaker(int alliance) {
  return new SequentialCommandGroup(
      InstantCommand([this]() { driveSubsystem.Drive(-1_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
}
Command* RobotContainer::MiddleTaxi(int alliance) {
  if (alliance == 1) {
    return new SequentialCommandGroup(
        InstantCommand([this]() { driveSubsystem.Drive(0.25_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
  } else {
    return new SequentialCommandGroup(
        InstantCommand([this]() { driveSubsystem.Drive(0.25_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
  }
}
Command* RobotContainer::MiddleTaxiPart2(int alliance) {
  return new SequentialCommandGroup(
      InstantCommand([this]() { driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
}
Command* RobotContainer::SideTaxi2(int alliance) {
  return new SequentialCommandGroup(
      InstantCommand([this]() { driveSubsystem.Drive(0.2_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
}
Command* RobotContainer::SideTaxi2Part2(int alliance) {
  if (alliance == 1) {
    return new SequentialCommandGroup(
        InstantCommand([this]() { driveSubsystem.Drive(0.25_mps, 0.433_mps, 0_rad_per_s, true, true); }, {}));
  } else {
    return new SequentialCommandGroup(
        InstantCommand([this]() { driveSubsystem.Drive(0.25_mps, -0.433_mps, 0_rad_per_s, true, true); }, {}));
  }
}

Command* RobotContainer::GetAutonomousCommand1(double xDir, double yDir, double rotation) {
  return new InstantCommand([this]() { driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, true, true); }, {});
}
