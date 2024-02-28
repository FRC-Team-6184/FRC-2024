// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : frontLeft{
        kFrontLeftDrivingCanId,
        kFrontLeftTurningCanId,
        kFrontLeftChassisAngularOffset
      },
      rearLeft{
        kRearLeftDrivingCanId,
        kRearLeftTurningCanId,
        kRearLeftChassisAngularOffset
      },
      frontRight{
        kFrontRightDrivingCanId,
        kFrontRightTurningCanId,
        kFrontRightChassisAngularOffset
      },
      rearRight{
        kRearRightDrivingCanId,
        kRearRightTurningCanId,
        kRearRightChassisAngularOffset
      },
      odometry{
        kDriveKinematics,
        frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
        {
          frontLeft.GetPosition(),
          frontRight.GetPosition(),
          rearLeft.GetPosition(),
          rearRight.GetPosition()
        },
        frc::Pose2d{}
      } {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  odometry.Update(frc::Rotation2d(
    units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
    {
      frontLeft.GetPosition(),
      rearLeft.GetPosition(),
      frontRight.GetPosition(),
      rearRight.GetPosition()
    }
  );
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           bool rateLimit) {
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(DriveConstants::kDirectionSlewRate / currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   currentTranslationDir);
    if (angleDif < 0.45 * std::numbers::pi) {
      currentTranslationDir = SwerveUtils::StepTowardsCircular(
          currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      currentTranslationMag = magLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        currentTranslationMag = magLimiter.Calculate(0.0);
      } else {
        currentTranslationDir =
            SwerveUtils::WrapAngle(currentTranslationDir + std::numbers::pi);
        currentTranslationMag = magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      currentTranslationDir = SwerveUtils::StepTowardsCircular(
          currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      currentTranslationMag = magLimiter.Calculate(0.0);
    }
    prevTime = currentTime;

    xSpeedCommanded = currentTranslationMag * cos(currentTranslationDir);
    ySpeedCommanded = currentTranslationMag * sin(currentTranslationDir);
    currentRotation = rotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      currentRotation * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{
                    gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  frontLeft.SetDesiredState(fl);
  frontRight.SetDesiredState(fr);
  rearLeft.SetDesiredState(bl);
  rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  frontLeft.SetDesiredState(desiredStates[0]);
  frontRight.SetDesiredState(desiredStates[1]);
  rearLeft.SetDesiredState(desiredStates[2]);
  rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  frontLeft.ResetEncoders();
  rearLeft.ResetEncoders();
  frontRight.ResetEncoders();
  rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(
             units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { gyro.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}

frc::Pose2d DriveSubsystem::GetPose() { return odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  odometry.ResetPosition(
      GetHeading(),
      {frontLeft.GetPosition(), frontRight.GetPosition(),
       rearLeft.GetPosition(), rearRight.GetPosition()},
      pose);
}
