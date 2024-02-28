#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

using ctre::phoenix6::hardware::TalonFX;
using frc2::SubsystemBase;

class ShooterSubsystem : public SubsystemBase {
 public:
  ShooterSubsystem();

  void Periodic() override;

  TalonFX shooter1;
  TalonFX shooter2;
};