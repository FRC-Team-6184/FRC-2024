// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Robot.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
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
using std::numbers::pi;
using std::string;
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
        }
        else if (driverController.GetLeftBumper()) {
            speedMultiplier = speedMode::TURBO_SPEED;
        }
        else if (driverController.GetRightBumper()) {
            speedMultiplier = speedMode::TURTLE_SPEED;
        }
        driveSubsystem.Drive(
          -meters_per_second_t{
            ApplyDeadband(driverController.GetLeftY() * speedMultiplier, OIConstants::driveDeadband)
          },
          -meters_per_second_t{
            ApplyDeadband(driverController.GetLeftX() * speedMultiplier, OIConstants::driveDeadband)
          },
          -radians_per_second_t{
            ApplyDeadband(driverController.GetRightX() * speedMultiplier, OIConstants::driveDeadband)
          },
          true,
          true
        );
      },
      {&driveSubsystem}
    )
  );
}

void RobotContainer::ConfigureButtonBindings() {
  //JoystickButton(&driverController, XboxController::Button::kRightBumper)
    //.WhileTrue(new RunCommand([this] { driveSubsystem.SetX(); }, {&driveSubsystem}));
}

void RobotContainer::leftAutoMode(bool orientation) {
  int polarity = 1;
  if (orientation) {
    polarity = -1;
  }
}

void RobotContainer::middleAutoMode() {}

void RobotContainer::rightAutoMode() {
  leftAutoMode(true);
}

Command* RobotContainer::GetAutonomousCommand(string autoMode) {
  // Set up config for trajectory
  TrajectoryConfig config(AutoConstants::maxSpeed, AutoConstants::maxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(driveSubsystem.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  
  if (autoMode == "Left") {
    leftAutoMode();
  }
  else if (autoMode == "Middle") {
    middleAutoMode();
  }
  else {
    rightAutoMode();
  }

  auto exampleTrajectory = TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
    Pose2d{0_m, 0_m, 0_deg},
    // Pass through these two interior waypoints, making an 's' curve path
    {},
    // End 3 meters straight ahead of where we started, facing forward
    Pose2d{0_m, 2_m, 0_deg},
    // Pass the config
    config
  );

  ProfiledPIDController<radians> thetaController{
    AutoConstants::pThetaController,
    0,
    0,
    AutoConstants::thetaControllerConstraints
  };

  thetaController.EnableContinuousInput(radian_t{-pi}, radian_t{pi});

  SwerveControllerCommand<4> swerveControllerCommand(
    exampleTrajectory, [this]() { return driveSubsystem.GetPose(); },

    driveSubsystem.kDriveKinematics,

    PIDController{AutoConstants::pXController, 0, 0},
    PIDController{AutoConstants::pYController, 0, 0},
    thetaController,

    [this](auto moduleStates) { driveSubsystem.SetModuleStates(moduleStates); },

    {&driveSubsystem}
  );

  // Reset odometry to the starting pose of the trajectory.
  driveSubsystem.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new SequentialCommandGroup(
    move(swerveControllerCommand),
    InstantCommand(
      [this]() {
        driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
      }, {}
    )
  );
}
