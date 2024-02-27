// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/PS5Controller.h"

#include "RobotContainer.h"
#include "Constants.h"

using ctre::phoenix6::hardware::TalonFX;
using ctre::phoenix6::controls::Follower;
class Robot : public frc::TimedRobot {
 public:
  TalonFX shooter1{ShooterConstants::shooter1CanId};
  TalonFX shooter2{ShooterConstants::shooter2CanId};

  PS5Controller shooterController{ShooterConstants::shooterControllerPort};

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  bool shooterOn;

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



};
