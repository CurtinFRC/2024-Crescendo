#include "vision/Limelight.h"

wom::vision::Limelight::Limelight(std::string *limelightName): _limelightName(limelightName) {}

std::string *wom::vision::Limelight::GetName() { return _limelightName; }

std::pair<double, double> wom::vision::Limelight::GetOffset() {
  std::pair<double, double> offset;

  offset.first = table->GetNumber("tx",0.0);
  offset.second = table->GetNumber("ty",0.0);

  return offset;
}

std::vector<double> wom::vision::Limelight::GetAprilTagData(std::string dataName) {
  return table->GetNumberArray(dataName ,std::vector<double>(6));
}

units::meters_per_second_t wom::vision::Limelight::GetSpeed(frc::Pose3d pose1, frc::Pose3d pose2, units::second_t dt) {
  frc::Transform3d dPose{pose1, pose2}; 
  frc::Translation3d dTranslation = dPose.Translation();

  units::meter_t y = dTranslation.Y();
  units::meter_t x = dTranslation.X();
  units::radian_t theta = units::math::atan(y / x);
  units::meter_t dTRANSLATION = x / units::math::cos(theta);
  return units::math::fabs(dTRANSLATION / dt);
}

frc::Pose3d wom::vision::Limelight::GetPose() {
  std::vector<double> pose = GetAprilTagData("botpose");
  return frc::Pose3d(pose[1] * 1_m, 1_m * pose[2], 1_m * pose[3], frc::Rotation3d(1_deg *(pose[4]), 1_deg *(pose[5]), 1_deg *(pose[6])));
}

void wom::vision::Limelight::OnStart() {
  std::cout << "starting" << std::endl;
}

void wom::vision::Limelight::OnUpdate(units::time::second_t dt) {
  wom::utils::WritePose3NT(table, GetPose());
}

bool wom::vision::Limelight::IsAtSetPoseVision(frc::Pose3d pose, units::second_t dt)  {
  frc::Pose3d actualPose = GetPose();
  frc::Pose3d relativePose = actualPose.RelativeTo(pose);
  return (units::math::fabs(relativePose.X()) < 0.01_m && units::math::fabs(relativePose.Y()) < 0.01_m && GetSpeed(pose, GetPose(), dt) < 1_m / 1_s);
}
