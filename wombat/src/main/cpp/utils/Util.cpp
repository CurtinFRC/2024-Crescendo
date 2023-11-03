#include "utils/Util.h"

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
    return val*val*(val > 0 ? 1 : -1);
}

