// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"
#include "RobotContainer.h"
#include "frc/AddressableLED.h"
#include "frc/PS5Controller.h"

using ctre::phoenix6::controls::Follower;
using ctre::phoenix6::hardware::TalonFX;
using rev::CANSparkFlex;
using rev::CANSparkMax;

class Robot : public frc::TimedRobot {
 public:
  TalonFX shooter1{ShooterConstants::shooter1CanId};
  TalonFX shooter2{ShooterConstants::shooter2CanId};
  frc::DigitalInput shooterLimitSwitch{ShooterConstants::shooterLimitSwitchId};
  // frc::DigitalInput shooterLoadedLimitSwitch{
  //     ShooterConstants::shooterLoadedLimitSwitchId};

  CANSparkMax pullThrough{ShooterConstants::pullThroughCanId,
                          CANSparkMax::MotorType::kBrushless};
  CANSparkMax shooterPivot{ShooterConstants::shooterPivotCanId,
                           CANSparkMax::MotorType::kBrushless};

  TalonFX LTelescopingArm{ShooterConstants::LTelescopingArmCanId};
  TalonFX RTelescopingArm{ShooterConstants::RTelescopingArmCanId};

  CANSparkFlex intakeWheel{IntakeConstants::intakeWheelCanId,
                           CANSparkFlex::MotorType::kBrushless};
  frc::DigitalInput intakeLimitSwitch{IntakeConstants::intakeLimitSwitchId};

  CANSparkMax intakePivot{IntakeConstants::intakePivotCanId,
                          CANSparkMax::MotorType::kBrushless};
  frc::DigitalInput pivotLimitSwitchUpper{
      IntakeConstants::pivotLimitSwitchUpperId};
  frc::DigitalInput pivotLimitSwitchLower{
      IntakeConstants::pivotLimitSwitchLowerId};

  XboxController driverController{OIConstants::driverControllerPort};
  PS5Controller shooterController{ShooterConstants::shooterControllerPort};

  frc::AddressableLED led{LedConstants::ledLightPort};
  std::array<frc::AddressableLED::LEDData, LedConstants::ledLength> ledBuffer;

  void intakeNote();
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
  int intakePivotDirection;
  bool autonomousRun = false;
  units::second_t autoTime;

  enum autoStates {
    moveToShooter,
    shootNote1,
    moveToNote,
    intakingNoteAutonomous,
    moveBackToShooter,
    shootNote2,
    taxi
  };

  struct {
    autoStates state = moveToShooter;
    autoStates lastTickState = moveToShooter;
    bool stateChange = true;
  } currentAuto;

  Color ledColor;

  enum noteAutoLoaderStates {
    off,
    intakeDeploying,
    intakingNote,
    intakeRetracting,
    shooterDeploying,
    noteLoading,
    noteLoaded,
    cancelling
  };

  struct {
    noteAutoLoaderStates state = off;
    units::second_t intakeDeployStartTime;
  } noteAutoLoaderAutomation;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* currentAutonomousCommand = nullptr;

  frc2::Command* autonomousCommand1 = nullptr;
  frc2::Command* autonomousCommand2 = nullptr;
  frc2::Command* autonomousCommand3 = nullptr;

  RobotContainer container;

  frc::SendableChooser<std::string> autoChooser;
  const std::string position1 = "Position 1";
  const std::string position2 = "Position 2";
  const std::string position3 = "Position 3";
  std::string selectedAuto;

  frc::SendableChooser<std::string> allianceChooser;
  const std::string redAlliance = "Red Alliance";
  const std::string blueAlliance = "Blue Alliance";
  std::string selectedAlliance;
  void automateNoteLoading();
};
