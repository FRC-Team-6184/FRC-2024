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
        if (driverController.GetLeftBumper() &&
            driverController.GetRightBumper()) {
          speedMultiplier = speedMode::LUDICROUS_SPEED;
        } else if (driverController.GetLeftBumper()) {
          speedMultiplier = speedMode::TURBO_SPEED;
        } else if (driverController.GetRightBumper()) {
          speedMultiplier = speedMode::TURTLE_SPEED;
        }
        driveSubsystem.Drive(
            meters_per_second_t{
                ApplyDeadband(-driverController.GetLeftY() * speedMultiplier,
                              OIConstants::driveDeadband)},
            meters_per_second_t{
                ApplyDeadband(-driverController.GetLeftX() * speedMultiplier,
                              OIConstants::driveDeadband)},
            radians_per_second_t{
                ApplyDeadband(-driverController.GetRightX() * speedMultiplier,
                              OIConstants::driveDeadband)},
            true, true);
      },
      {&driveSubsystem}));
}

void RobotContainer::ConfigureButtonBindings() {
  // JoystickButton(&driverController, XboxController::Button::kRightBumper)
  //.WhileTrue(new RunCommand([this] { driveSubsystem.SetX(); },
  //{&driveSubsystem}));
}

Command* RobotContainer::GetAutonomousCommand1(double xDir, double yDir,
                                               double rotation) {
  // Set up config for trajectory
  TrajectoryConfig config(AutoConstants::maxSpeed,
                          AutoConstants::maxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(driveSubsystem.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  // int multiplier = 1;
  // if (alliance == "Blue Alliance") {
  //   multiplier = -1;
  // }

  frc::Trajectory trajectory1;

  // if (position == "Position 1") {
  //   trajectory1 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{0_m, 0_m, 0_deg},
  //       // Pass through these two interior waypoints, making an 's' curve
  //       path
  //       {},

  //       Pose2d{0_m, 3_m, 0_deg},

  //       // Pass the config
  //       config);
  // } else if (position == "Position 2") {
  //   trajectory1 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{0_m, 0_m, 0_deg},
  //       // Pass through these interior waypoints
  //       {},

  //       Pose2d{0_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);
  // } else {
  //   trajectory1 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{0_m, 0_m, 0_deg},
  //       // Pass through these interior waypoints
  //       {}, Pose2d{-0.16_m, 1.45_m, 0_deg},
  //       // Pass the config
  //       config);
  // }

  // trajectory1 = TrajectoryGenerator::GenerateTrajectory(
  //     Pose2d{0_m, 0_m, 0_deg}, {}, Pose2d{2_m, 0_m, 0_deg}, config);
  ProfiledPIDController<radians> thetaController{
      AutoConstants::pThetaController, 0, 0,
      AutoConstants::thetaControllerConstraints};

  thetaController.EnableContinuousInput(radian_t{-pi}, radian_t{pi});

  SwerveControllerCommand<4> swerveControllerCommand(
      trajectory1, [this]() { return driveSubsystem.GetPose(); },

      driveSubsystem.kDriveKinematics,

      PIDController{AutoConstants::pXController, 0, 0},
      PIDController{AutoConstants::pYController, 0, 0}, thetaController,

      [this](auto moduleStates) {
        driveSubsystem.SetModuleStates(moduleStates);
      },

      {&driveSubsystem});

  // Reset odometry to the starting pose of the trajectory.
  driveSubsystem.ResetOdometry(trajectory1.InitialPose());

  // no auto
  return new InstantCommand(
      [this]() { driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, true, true); },
      {});
}

Command* RobotContainer::GetAutonomousCommand2(double xDir, double yDir,
                                               double rotation) {
  // Set up config for trajectory
  TrajectoryConfig config(AutoConstants::maxSpeed,
                          AutoConstants::maxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(driveSubsystem.kDriveKinematics);

  // int multiplier = 1;
  // if (alliance == "Blue Alliance") {
  //   multiplier = -1;
  // }

  frc::Trajectory trajectory2;

  // if (position == "Position 1") {
  //   trajectory2 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{-1_m, -multiplier * 0.657_m, multiplier * 60_deg},
  //       // Pass through these two interior waypoints, making an 's' curve
  //       path
  //       {},
  //       // End 3 meters straight ahead of where we started, facing forward
  //       Pose2d{1_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);
  // } else if (position == "Position 2") {
  //   trajectory2 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{0.3_m, 0_m, 0_deg},
  //       // Pass through these interior waypoints
  //       {}, Pose2d{1_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);
  // } else {
  //   trajectory2 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{-1_m, multiplier * 0.657_m, -multiplier * 60_deg},
  //       // Pass through these interior waypoints
  //       {}, Pose2d{1_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);
  // }

  ProfiledPIDController<radians> thetaController{
      AutoConstants::pThetaController, 0, 0,
      AutoConstants::thetaControllerConstraints};

  thetaController.EnableContinuousInput(radian_t{-pi}, radian_t{pi});

  SwerveControllerCommand<4> swerveControllerCommand(
      trajectory2, [this]() { return driveSubsystem.GetPose(); },

      driveSubsystem.kDriveKinematics,

      PIDController{AutoConstants::pXController, 0, 0},
      PIDController{AutoConstants::pYController, 0, 0}, thetaController,

      [this](auto moduleStates) {
        driveSubsystem.SetModuleStates(moduleStates);
      },

      {&driveSubsystem});

  // Reset odometry to the starting pose of the trajectory.
  driveSubsystem.ResetOdometry(trajectory2.InitialPose());

  // no auto
  return new SequentialCommandGroup(
      move(swerveControllerCommand),
      InstantCommand(
          [this]() {
            driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
          },
          {}));
}

Command* RobotContainer::GetAutonomousCommand3(double xDir, double yDir,
                                               double rotation) {
  // Set up config for trajectory
  TrajectoryConfig config(AutoConstants::maxSpeed,
                          AutoConstants::maxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(driveSubsystem.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.

  // int multiplier = 1;
  // if (alliance == "Blue Alliance") {
  //   multiplier = -1;
  // }

  frc::Trajectory trajectory3;

  // if (position == "Position 1") {
  //   trajectory3 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{1_m, 0_m, 0_deg},
  //       // Pass through these two interior waypoints, making an 's' curve
  //       path
  //       {},
  //       // End 3 meters straight ahead of where we started, facing forward
  //       Pose2d{1_m, 0_m, 90_deg},
  //       // Pass the config
  //       config);
  // } else if (position == "Position 2") {
  //   trajectory3 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{1_m, 0_m, 0_deg},
  //       // Pass through these interior waypoints
  //       {}, Pose2d{-0.38_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);
  // } else {
  //   trajectory3 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{1_m, 0_m, 0_deg},
  //       // Pass through these interior waypoints
  //       {}, Pose2d{-1_m, multiplier * 0.657_m, -multiplier * 60_deg},
  //       // Pass the config
  //       config);
  // }

  ProfiledPIDController<radians> thetaController{
      AutoConstants::pThetaController, 0, 0,
      AutoConstants::thetaControllerConstraints};

  thetaController.EnableContinuousInput(radian_t{-pi}, radian_t{pi});

  SwerveControllerCommand<4> swerveControllerCommand(
      trajectory3, [this]() { return driveSubsystem.GetPose(); },

      driveSubsystem.kDriveKinematics,

      PIDController{AutoConstants::pXController, 0, 0},
      PIDController{AutoConstants::pYController, 0, 0}, thetaController,

      [this](auto moduleStates) {
        driveSubsystem.SetModuleStates(moduleStates);
      },

      {&driveSubsystem});

  // Reset odometry to the starting pose of the trajectory.
  driveSubsystem.ResetOdometry(trajectory3.InitialPose());

  // no auto
  return new SequentialCommandGroup(
      move(swerveControllerCommand),
      InstantCommand(
          [this]() {
            driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
          },
          {}));
}

Command* RobotContainer::GetAutonomousCommand4(double xDir, double yDir,
                                               double rotation) {
  // Set up config for trajectory
  TrajectoryConfig config(AutoConstants::maxSpeed,
                          AutoConstants::maxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(driveSubsystem.kDriveKinematics);

  // int multiplier = 1;
  // if (alliance == "Blue Alliance") {
  //   multiplier = -1;
  // }

  frc::Trajectory trajectory4;

  // if (position == "Position 1") {
  //   trajectory4 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{-1_m, -multiplier * 0.657_m, multiplier * 60_deg},
  //       // Pass through these two interior waypoints, making an 's' curve
  //       path {Translation2d{0_m, multiplier * 3_m}},
  //       // End 3 meters straight ahead of where we started, facing forward
  //       Pose2d{4_m, multiplier * 3_m, 0_deg},
  //       // Pass the config
  //       config);
  // } else if (position == "Position 2") {
  //   trajectory4 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{-0.38_m, 0_m, 0_deg},
  //       // Pass through these interior waypoints
  //       {Translation2d{1_m, -multiplier * 0.5_m}},
  //       Pose2d{4_m, -multiplier * 0.5_m, 0_deg},
  //       // Pass the config
  //       config);
  // } else {
  //   trajectory4 = TrajectoryGenerator::GenerateTrajectory(
  //       // Start at the origin facing the +X direction
  //       Pose2d{-1_m, multiplier * 0.657_m, -multiplier * 60_deg},
  //       // Pass through these interior waypoints
  //       {Translation2d{1_m, 0_m}}, Pose2d{4_m, 0_m, 0_deg},
  //       // Pass the config
  //       config);
  // }

  ProfiledPIDController<radians> thetaController{
      AutoConstants::pThetaController, 0, 0,
      AutoConstants::thetaControllerConstraints};

  thetaController.EnableContinuousInput(radian_t{-pi}, radian_t{pi});

  SwerveControllerCommand<4> swerveControllerCommand(
      trajectory4, [this]() { return driveSubsystem.GetPose(); },

      driveSubsystem.kDriveKinematics,

      PIDController{AutoConstants::pXController, 0, 0},
      PIDController{AutoConstants::pYController, 0, 0}, thetaController,

      [this](auto moduleStates) {
        driveSubsystem.SetModuleStates(moduleStates);
      },

      {&driveSubsystem});

  // Reset odometry to the starting pose of the trajectory.
  driveSubsystem.ResetOdometry(trajectory4.InitialPose());

  // no auto
  return new SequentialCommandGroup(
      move(swerveControllerCommand),
      InstantCommand(
          [this]() {
            driveSubsystem.Drive(0_mps, 0_mps, 0_rad_per_s, false, false);
          },
          {}));
}