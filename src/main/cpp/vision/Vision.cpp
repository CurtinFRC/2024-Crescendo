// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/Vision.h"

FMAP::FMAP(std::string path) : _path(path) {
  std::cout << "Parsing FMAP" << std::endl;

  deploy_dir = frc::filesystem::GetDeployDirectory();

  std::ifstream file(deploy_dir + "/" + _path);

  // parse the json file into a wpi::json object

  wpi::json j;

  file >> j;

  // iterate through the json object and add each tag to the vector

  for (auto& element : j["fiducials"]) {
    AprilTag tag;

    tag.id = element["id"];
    tag.size = element["size"];
    tag.transform[0][0] = element["transform"][0];
    tag.transform[0][1] = element["transform"][1];
    tag.transform[0][2] = element["transform"][2];
    tag.transform[0][3] = element["transform"][3];
    tag.transform[1][0] = element["transform"][4];
    tag.transform[1][1] = element["transform"][5];
    tag.transform[1][2] = element["transform"][6];
    tag.transform[1][3] = element["transform"][7];
    tag.transform[2][0] = element["transform"][8];
    tag.transform[2][1] = element["transform"][9];
    tag.transform[2][2] = element["transform"][10];
    tag.transform[2][3] = element["transform"][11];
    tag.transform[3][0] = element["transform"][12];
    tag.transform[3][1] = element["transform"][13];
    tag.transform[3][2] = element["transform"][14];
    tag.transform[3][3] = element["transform"][15];
    if (element["unique"] == 1) {
      tag.unique = true;
    } else {
      tag.unique = false;
    }

    _tags.push_back(tag);
  }

  file.close();

  std::cout << "Loaded " << _tags.size() << " tags" << std::endl;

  for (int i = 0; i < _tags.size(); i++) {
    std::cout << "Tag " << _tags[i].id << " is " << _tags[i].size << "m" << std::endl;
  }

  std::cout << "Loaded FMAP" << std::endl;
}

std::vector<AprilTag> FMAP::GetTags() {
  return _tags;
}

Vision::Vision(std::string name, FMAP fmap) : _fmap(fmap) {
  _limelight = new wom::Limelight(name);

  _limelight->SetPipeline(wom::LimelightPipeline::kPipeline0);
}

frc::Pose3d Vision::GetPose() {
  return _limelight->GetPose();
}

std::pair<units::meter_t, units::degree_t> Vision::GetDistanceToTarget(VisionTarget target) {
  SetMode(VisionModes::kAprilTag);

  std::vector<AprilTag> tags = _fmap.GetTags();

  frc::Pose3d pose = _limelight->GetPose();

  for (int i = 0; i < tags.size(); i++) {
    if (tags[i].id == static_cast<int>(target)) {
      frc::Pose3d aprilTagPos =
          frc::Pose3d(tags[i].transform[0][3] * 1_m, tags[i].transform[1][3] * 1_m,
                      tags[i].transform[2][3] * 1_m, frc::Rotation3d(0_deg, 0_deg, 0_deg));

      units::meter_t distance = pose.Translation().Distance(aprilTagPos.Translation());

      units::degree_t angle = units::math::atan((pose.X() - units::meter_t{tags[i].transform[0][3]}) /
                                                (pose.Y() - units::meter_t{tags[i].transform[1][3]}));

      return std::make_pair(distance, angle);
    }
  }

  return std::make_pair(0_m, 0_deg);
}

std::pair<units::meter_t, units::degree_t> Vision::GetDistanceToTarget(int id) {
  if (id < APRILTAGS_MIN || id > APRILTAGS_MAX) {
    return std::make_pair(0_m, 0_deg);
  }

  SetMode(VisionModes::kAprilTag);

  std::vector<AprilTag> tags = _fmap.GetTags();

  for (int i = 0; i < tags.size(); i++) {
    if (tags[i].id == id) {
      // get distance to the limelight
      frc::Pose3d currentPose = _limelight->GetPose();
      frc::Pose3d aprilTagPos =
          frc::Pose3d(tags[i].transform[0][3] * 1_m, tags[i].transform[1][3] * 1_m,
                      tags[i].transform[2][3] * 1_m, frc::Rotation3d(0_deg, 0_deg, 0_deg));

      units::meter_t distance = currentPose.Translation().Distance(aprilTagPos.Translation());

      units::degree_t angle = units::math::atan((currentPose.X() - units::meter_t{tags[i].transform[0][3]}) /
                                                (currentPose.Y() - units::meter_t{tags[i].transform[1][3]}));

      return std::make_pair(distance, angle);
    }
  }

  return std::make_pair(0_m, 0_deg);
}

std::vector<AprilTag> Vision::GetTags() {
  return _fmap.GetTags();
}

frc::Pose2d Vision::AlignToTarget(VisionTarget target, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);

  units::meter_t x = distance.first * units::math::cos(distance.second);
  units::meter_t y = distance.first * units::math::sin(distance.second);

  frc::Pose2d pose = frc::Pose2d(x, y, distance.second);

  swerveDrive->SetPose(pose);

  return pose;
}

bool Vision::IsAtPose(frc::Pose3d pose, units::second_t dt) {
  return _limelight->IsAtSetPoseVision(pose, dt);
}

void Vision::SetMode(VisionModes mode) {
  switch (mode) {
    case VisionModes::kAprilTag: {
      _limelight->SetPipeline(wom::LimelightPipeline::kPipeline0);
      break;
    }
    case VisionModes::kRing: {
      _limelight->SetPipeline(wom::LimelightPipeline::kPipeline1);
      break;
    }
  }
}

bool Vision::TargetIsVisible(VisionTargetObjects target) {
  switch (target) {
    case VisionTargetObjects::kNote: {
      SetMode(VisionModes::kRing);
      break;
    }
  }

  return _limelight->HasTarget();
}

int Vision::CurrentAprilTag() {
  SetMode(VisionModes::kAprilTag);

  return _limelight->GetTargetingData(wom::LimelightTargetingData::kTid);
}
