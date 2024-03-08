// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "sim/Sim.h"

#include "drivetrain/SwerveDrive.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTableInstance.h"
#include "utils/Util.h"

wom::sim::SimSwerve::SimSwerve(wom::drivetrain::SwerveDrive* _swerve) : _swerve(_swerve) {
  frc::SmartDashboard::PutData("Field", &_field);
}

void wom::sim::SimSwerve::OnTick() {
  _field.SetRobotPose(_swerve->GetPose());
  utils::WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("swerveSetpoint"),
                      _swerve->GetSetpoint());
}

void wom::sim::SimSwerve::OnTick(frc::Pose2d pose) {
  _field.SetRobotPose(pose);
}
