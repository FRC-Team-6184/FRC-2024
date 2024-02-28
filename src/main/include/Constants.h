// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

using frc::TrapezoidProfile;
using units::radians;

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  constexpr units::meters_per_second_t maxSpeed = 4.8_mps;
  constexpr units::radians_per_second_t maxAngularSpeed{2 * std::numbers::pi};

  constexpr double directionSlewRate = 1.2;   // radians per second
  constexpr double magnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
  constexpr double rotationalSlewRate = 2.0;  // percent per second (1 = 100%)

  // Chassis configuration
  constexpr units::meter_t trackWidth = 0.635_m;  // Distance between centers of right and left wheels on robot
  constexpr units::meter_t wheelBase = 0.635_m;  // Distance between centers of front and back wheels on robot

  // Angular offsets of the modules relative to the chassis in radians
  constexpr double frontLeftChassisAngularOffset = -std::numbers::pi / 2;
  constexpr double frontRightChassisAngularOffset = 0;
  constexpr double rearLeftChassisAngularOffset = std::numbers::pi;
  constexpr double rearRightChassisAngularOffset = std::numbers::pi / 2;

  // SPARK MAX CAN IDs
  constexpr int frontLeftTurningCanId = 1;
  constexpr int frontLeftDrivingCanId = 2;
  constexpr int rearLeftTurningCanId = 3;
  constexpr int rearLeftDrivingCanId = 4;
  constexpr int rearRightTurningCanId = 5;
  constexpr int rearRightDrivingCanId = 6;
  constexpr int frontRightTurningCanId = 7;
  constexpr int frontRightDrivingCanId = 8;
}

namespace ShooterConstants {
  constexpr int shooter1CanId = 9;
  constexpr int shooter2CanId = 10;

  constexpr int shooterControllerPort = 1;
}

namespace IntakeConstants {
  constexpr int intakeWheelCanId = 11;
}

namespace ModuleConstants {
  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of the steering motor in the MAXSwerve Module.
  constexpr bool turningEncoderInverted = true;

  // The MAXSwerve module can be configured with one of three pinion gears: 12T,
  // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
  // more teeth will result in a robot that drives faster).
  constexpr int drivingMotorPinionTeeth = 14;

  // Calculations required for driving motor conversion factors and feed forward
  constexpr double drivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM
  constexpr units::meter_t wheelDiameter = 0.0762_m;
  constexpr units::meter_t wheelCircumference = wheelDiameter * std::numbers::pi;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  // teeth on the bevel pinion
  constexpr double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
  constexpr double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumference.value()) / drivingMotorReduction;

  constexpr double drivingEncoderPositionFactor =
    (wheelDiameter.value() * std::numbers::pi) / drivingMotorReduction; // meters
  constexpr double kDrivingEncoderVelocityFactor =
    ((wheelDiameter.value() * std::numbers::pi) / drivingMotorReduction) / 60.0;  // meters per second

  constexpr double turningEncoderPositionFactor = (2 * std::numbers::pi);  // radians
  constexpr double turningEncoderVelocityFactor = (2 * std::numbers::pi) / 60.0;  // radians per second

  constexpr units::radian_t turningEncoderPositionPIDMinInput = 0_rad;
  constexpr units::radian_t turningEncoderPositionPIDMaxInput = units::radian_t{turningEncoderPositionFactor};

  constexpr double drivingP = 0.04;
  constexpr double drivingI = 0;
  constexpr double drivingD = 0;
  constexpr double drivingFF = (1 / driveWheelFreeSpeedRps);
  constexpr double drivingMinOutput = -1;
  constexpr double drivingMaxOutput = 1;

  constexpr double turningP = 1;
  constexpr double turningI = 0;
  constexpr double turningD = 0;
  constexpr double turningFF = 0;
  constexpr double turningMinOutput = -1;
  constexpr double turningMaxOutput = 1;

  constexpr rev::CANSparkMax::IdleMode drivingMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;
  constexpr rev::CANSparkMax::IdleMode turningMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

  constexpr units::ampere_t drivingMotorCurrentLimit = 40_A;
  constexpr units::ampere_t turningMotorCurrentLimit = 30_A;
}

namespace AutoConstants {
  constexpr auto maxSpeed = 1_mps;
  constexpr auto maxAcceleration = 1_mps_sq;
  constexpr auto maxAngularSpeed = 1_rad_per_s;
  constexpr auto maxAngularAcceleration = 1_rad_per_s_sq;

  constexpr double pXController = 0.5;
  constexpr double pYController = 0.5;
  constexpr double pThetaController = 0.5;

  extern const TrapezoidProfile<radians>::Constraints thetaControllerConstraints;
}

namespace OIConstants {
  constexpr int driverControllerPort = 0;
  constexpr double driveDeadband = 0.05;
}

namespace speedMode {
  constexpr double TURTLE_SPEED = 0.25;
  constexpr double NORMAL_SPEED = .5;
  constexpr double TURBO_SPEED = .75;
  constexpr double LUDICROUS_SPEED = 1;
}