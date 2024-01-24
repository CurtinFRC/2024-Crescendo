// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/Util.h"

#include <frc/RobotController.h>

#include "units/voltage.h"

units::second_t wom::utils::now() {
  uint64_t now = frc::RobotController::GetFPGATime();
  return static_cast<double>(now) / 1000000 * 1_s;
}

void wom::utils::WritePose2NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose2d pose) {
  table->GetEntry("x").SetDouble(pose.X().value());
  table->GetEntry("y").SetDouble(pose.Y().value());
  table->GetEntry("angle").SetDouble(pose.Rotation().Degrees().value());
}

void wom::utils::WritePose3NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose3d pose) {
  table->GetEntry("x").SetDouble(pose.X().value());
  table->GetEntry("y").SetDouble(pose.Y().value());
  table->GetEntry("z").SetDouble(pose.Z().value());

  table->GetEntry("angle").SetDouble(pose.Rotation().Z().convert<units::degree>().value());
}

double wom::utils::deadzone(double val, double deadzone) {
  return std::fabs(val) > deadzone ? val : 0;
}

double wom::utils::spow2(double val) {
  return val * val * (val > 0 ? 1 : -1);
}

units::volt_t wom::utils::LimitVoltage(units::volt_t voltage) {
  if (voltage >= frc::RobotController::GetBatteryVoltage()) {
    return frc::RobotController::GetBatteryVoltage() - 0.5_V;
  }

  return voltage;
}

units::volt_t wom::utils::GetVoltage(frc::MotorController* controller) {
  return controller->Get() * frc::RobotController::GetBatteryVoltage();
}

void wom::utils::WriteTrajectory(std::shared_ptr<nt::NetworkTable> table, frc::Trajectory trajectory) {
  table->GetEntry("length").SetDouble(trajectory.TotalTime().value());

  // write the trajectory to the network table
  int i = 0;
  for (auto state : trajectory.States()) {
    table->GetSubTable(std::to_string(i))->GetEntry("x").SetDouble(state.pose.X().value());
    table->GetSubTable(std::to_string(i))->GetEntry("y").SetDouble(state.pose.Y().value());
    table->GetSubTable(std::to_string(i))
        ->GetEntry("angle")
        .SetDouble(state.pose.Rotation().Degrees().value());
    table->GetSubTable(std::to_string(i))->GetEntry("time").SetDouble(state.t.value());

    i++;
  }
}

void wom::utils::WriteTrajectoryState(std::shared_ptr<nt::NetworkTable> table, frc::Trajectory::State state) {
  table->GetEntry("x").SetDouble(state.pose.X().value());
  table->GetEntry("y").SetDouble(state.pose.Y().value());
  table->GetEntry("angle").SetDouble(state.pose.Rotation().Degrees().value());
  table->GetEntry("time").SetDouble(state.t.value());
}

frc::Pose2d wom::utils::TrajectoryStateToPose2d(frc::Trajectory::State state) {
  frc::Pose2d pose = state.pose;

  return pose;
}
