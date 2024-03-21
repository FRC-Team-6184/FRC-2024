// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkFlex.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/PS5Controller.h"

#include "RobotContainer.h"
#include "Constants.h"
#include "frc/AddressableLED.h"

using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::controls::Follower;
using rev::CANSparkMax;
using rev::CANSparkFlex;

class Robot : public frc::TimedRobot {
 public:
  TalonFX shooter1{ShooterConstants::shooter1CanId};
  TalonFX shooter2{ShooterConstants::shooter2CanId};
  frc::DigitalInput shooterLimitSwitch{ShooterConstants::shooterLimitSwitchId};
  frc::DigitalInput shooterLoadedLimitSwitch{ShooterConstants::shooterLoadedLimitSwitchId};

  CANSparkMax pullThrough{ShooterConstants::pullThroughCanId, CANSparkMax::MotorType::kBrushless};
  CANSparkMax shooterPivot{ShooterConstants::shooterPivotCanId, CANSparkMax::MotorType::kBrushless};

  TalonFX LTelescopingArm{ShooterConstants::LTelescopingArmCanId};
  TalonFX RTelescopingArm{ShooterConstants::RTelescopingArmCanId};

  CANSparkFlex intakeWheel{IntakeConstants::intakeWheelCanId, CANSparkFlex::MotorType::kBrushless};
  frc::DigitalInput intakeLimitSwitch {IntakeConstants::intakeLimitSwitchId};

  CANSparkMax intakePivot{IntakeConstants::intakePivotCanId, CANSparkMax::MotorType::kBrushless};
  frc::DigitalInput pivotLimitSwitchUpper{IntakeConstants::pivotLimitSwitchUpperId};
  frc::DigitalInput pivotLimitSwitchLower{IntakeConstants::pivotLimitSwitchLowerId};
  
  XboxController driverController{OIConstants::driverControllerPort};
  PS5Controller shooterController{ShooterConstants::shooterControllerPort};

  frc::AddressableLED led{LedConstants::ledLightPort};
  std::array<frc::AddressableLED::LEDData, LedConstants::ledLength> ledBuffer;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  units::second_t intakeTime;
  units::second_t shooterTime;
  bool intakeOn;
  int intakeDirection;
  bool pullThroughOn;
  int pullThroughDirection;
  double telescopingArmDir;
  int pastPOV;
  
  Color ledColor;

  enum noteAutoLoaderStates { off, intakeDeploying, intakingNote, intakeRetracting, shooterDeploying, noteLoading, noteLoaded, cancelling };

  struct {
    noteAutoLoaderStates state = off;
    units::second_t intakeDeployStartTime;
  } noteAutoLoaderAutomation;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* autonomousCommand = nullptr;

  RobotContainer container;

  frc::SendableChooser<std::string> autoChooser;
  const std::string leftAuto = "Left";
  const std::string middleAuto = "Middle";
  const std::string rightAuto = "Right";
  std::string selectedAuto;

  void automateNoteLoading();
};
