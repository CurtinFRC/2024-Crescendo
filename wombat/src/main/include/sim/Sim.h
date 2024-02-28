// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "drivetrain/SwerveDrive.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/Field2d.h"

namespace wom {
namespace sim {
class SimSwerve {
 public:
  explicit SimSwerve(drivetrain::SwerveDrive* _swerve);

  void OnTick();
  void OnTick(frc::Pose2d pose);

 private:
  frc::Field2d _field;
  drivetrain::SwerveDrive* _swerve;
};
}  // namespace sim
}  // namespace wom
