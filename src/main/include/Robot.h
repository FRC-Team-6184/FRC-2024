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
  frc::DigitalInput shooterLoadedLimitSwitch{ShooterConstants::shooterLoadedLimitSwitchId};

  CANSparkMax pullThrough{ShooterConstants::pullThroughCanId, CANSparkMax::MotorType::kBrushless};
  CANSparkMax shooterPivot{ShooterConstants::shooterPivotCanId, CANSparkMax::MotorType::kBrushless};

  TalonFX LTelescopingArm{ShooterConstants::LTelescopingArmCanId};
  TalonFX RTelescopingArm{ShooterConstants::RTelescopingArmCanId};

  CANSparkFlex intakeWheel{IntakeConstants::intakeWheelCanId, CANSparkFlex::MotorType::kBrushless};
  frc::DigitalInput intakeLimitSwitch{IntakeConstants::intakeLimitSwitchId};

  CANSparkMax intakePivot{IntakeConstants::intakePivotCanId, CANSparkMax::MotorType::kBrushless};
  frc::DigitalInput pivotLimitSwitchUpper{IntakeConstants::pivotLimitSwitchUpperId};
  frc::DigitalInput pivotLimitSwitchLower{IntakeConstants::pivotLimitSwitchLowerId};

  XboxController driverController{OIConstants::driverControllerPort};
  PS5Controller shooterController{ShooterConstants::shooterControllerPort};

  frc::AddressableLED led{LedConstants::ledLightPort};
  std::array<frc::AddressableLED::LEDData, LedConstants::ledLength> ledBuffer;

  frc::Trajectory trajectory;

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
  int shooterDir;
  bool intakPivotOverride;
  units::second_t shooterIntakeTime;

  enum autoStates {
    autoOff,
    runningAuto1,
    runningAuto2,
    runningAuto3,
    runningAuto4,
    intakingNoteAutonomous,
    shootingNote,
    taxi
  };

  struct {
    autoStates state = autoOff;
    autoStates lastTickState = autoOff;
    bool twoNote = false;
  } currentAuto;

  Color ledColor;

  enum noteAutoLoaderStates {
    off,
    intakeDeploying,
    intakingNote,
    intakeRetracting,
    noteLoading,
    noteLoaded,
    cancelling
  };

  struct {
    noteAutoLoaderStates state = off;
    units::second_t intakeDeployStartTime;
    double telescopingArmOffset;
  } noteAutoLoaderAutomation;

  bool shooterIntakingNote = false;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* currentAutonomousCommand = nullptr;

  frc2::Command* autonomousCommand1 = nullptr;
  frc2::Command* autonomousCommand2 = nullptr;
  frc2::Command* autonomousCommand3 = nullptr;
  frc2::Command* autonomousCommand4 = nullptr;

  RobotContainer container;

  frc::SendableChooser<std::string> autoChooser;
  const std::string position1 = "Position 1 (Source Side)";
  const std::string position2 = "Position 2 (Middle; facing forward)";
  const std::string position3 = "Position 3 (Amp Side)";
  std::string selectedAuto;

  frc::SendableChooser<std::string> allianceChooser;
  const std::string redAlliance = "Red Alliance";
  const std::string blueAlliance = "Blue Alliance";
  std::string selectedAlliance;
  void automateNoteLoading();
  std::string noteAutoLoaderStateString();
  void initializeShuffleBoard();
  void populateShuffleBoard();
};
