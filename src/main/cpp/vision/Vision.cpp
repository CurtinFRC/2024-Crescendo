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

  std::cout << j["fiducials"] << std::endl;

  // iterate through the json object and add each tag to the vector

  for (auto& fiducial : j["fiducials"]) {
    std::cout << "Loading AprilTag " << fiducial["id"] << std::endl;
      AprilTag tag;
      tag.id = fiducial["id"];
      tag.size = fiducial["size"];
      if (fiducial["unique"] == 1) {
          tag.unique = true;
      } else {
          tag.unique = false;
      }

      const auto& transform = fiducial["transform"];
      for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
               tag.transform[i][j] = transform[i * 4 + j];
          }
      }

      tag.yaw = units::radian_t{tag.transform[0][0]};
      tag.pitch = units::radian_t{tag.transform[1][1]};
      tag.roll = units::radian_t{tag.transform[2][2]};

      tag.pos = frc::Pose3d(units::meter_t{tag.transform[0][3]}, units::meter_t{tag.transform[1][3]}, units::meter_t{tag.transform[2][3]}, frc::Rotation3d(0_deg, 0_deg, 0_deg));

       _tags.push_back(tag);
    }

  file.close();

  std::cout << "Loaded " << _tags.size() << " tags" << std::endl;

  for (int i = 0; i < _tags.size(); i++) {
    std::cout << "Tag " << _tags[i].id << " is at: X: " << _tags[i].pos.X().value() << " Y: " << _tags[i].pos.Y().value()
              << " Z: " << _tags[i].pos.Z().value() << std::endl;
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

  for (int i = 0; i < tags.size(); i++) {
    if (tags[i].id == static_cast<int>(target)) {
      AprilTag tag = tags[i];
      frc::Pose3d pose = _limelight->GetPose();
      //frc::Pose3d pose = frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0_deg, 0_deg, 0_deg));

      units::meter_t distance = pose.Translation().Distance(tag.pos.Translation());

      units::degree_t angle = units::math::atan((pose.X() - tag.pos.X()) / (pose.Y() - tag.pos.Y()));

      std::cout << "Distance: " << distance.value() << " Angle: " << angle.value() << std::endl;

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
      AprilTag tag = tags[i];
      frc::Pose3d pose = _limelight->GetPose();
      //frc::Pose3d pose = frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0_deg, 0_deg, 0_deg));

      // units::meter_t distance = pose.Translation().Distance(tag.pos.Translation());
      units::meter_t distance = units::meter_t{std::sqrt(std::pow(tag.pos.X().value(), 2) + std::pow(tag.pos.Y().value(), 2))};

      units::degree_t angle = units::math::atan((pose.X() - tag.pos.X()) / (pose.Y() - tag.pos.Y()));

      //std::cout << "Distance: " << distance.value() << " Angle: " << angle.value() << std::endl;

      return std::make_pair(distance, angle);
    }
  }

  return std::make_pair(0_m, 0_deg);
}

std::vector<AprilTag> Vision::GetTags() {
  return _fmap.GetTags();
}

frc::Pose2d Vision::AlignToTarget(VisionTarget target, units::meter_t offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);

  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  //frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);
  
  units::meter_t x_offset = distance.first * units::math::cos(distance.second);
  units::meter_t y_offset = distance.first * units::math::sin(distance.second);

  // reduce both offsets by the offset parameter (in relative amount)
  x_offset -= offset * units::math::cos(distance.second);
  y_offset -= offset * units::math::sin(distance.second);

  frc::Pose2d pose = frc::Pose2d(y_offset, x_offset, distance.second);
  

  // Print the results
  std::cout << "Aligning to tag " << static_cast<int>(target) << ": X: " << pose.X().value() << " Y: " << pose.Y().value() << " Angle: " << pose.Rotation().Degrees().value() << ". At offset " << offset.value() << "." << std::endl;

  // Set the new pose to the swerve drive
  swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::AlignToTarget(int target, units::meter_t offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  // Get distance to target
  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);

  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  //frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);
  
  units::meter_t x_offset = distance.first * units::math::cos(distance.second);
  units::meter_t y_offset = distance.first * units::math::sin(distance.second);

  // reduce both offsets by the offset parameter (in relative amount)
  x_offset -= offset * units::math::cos(distance.second);
  y_offset -= offset * units::math::sin(distance.second);

  frc::Pose2d pose = frc::Pose2d(y_offset, x_offset, distance.second);
  

  // Print the results
  std::cout << "Aligning to tag " << target << ": X: " << pose.X().value() << " Y: " << pose.Y().value() << " Angle: " << pose.Rotation().Degrees().value() << ". At offset " << offset.value() << "." << std::endl;

  // Set the new pose to the swerve drive
  //swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::AlignToTarget(VisionTarget target, frc::Translation2d offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);

  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  //frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);
  
  units::meter_t x_offset = distance.first * units::math::cos(distance.second);
  units::meter_t y_offset = distance.first * units::math::sin(distance.second);

  // reduce both offsets by the offset parameter (in relative amount)
  //x_offset -= offset * units::math::cos(distance.second);
  //y_offset -= offset * units::math::sin(distance.second);

  if (offset.X() > 0_m) {
    x_offset -= offset.X() * units::math::cos(distance.second);
  } else {
    x_offset += offset.X() * units::math::cos(distance.second);
  }

  if (offset.Y() > 0_m) {
    y_offset -= offset.Y() * units::math::sin(distance.second);
  } else {
    y_offset += offset.Y() * units::math::sin(distance.second);
  }

  frc::Pose2d pose = frc::Pose2d(y_offset, x_offset, distance.second);
  

  // Print the results
  std::cout << "Aligning to tag " << static_cast<int>(target) << ": X: " << pose.X().value() << " Y: " << pose.Y().value() << " Angle: " << pose.Rotation().Degrees().value() << ". At offset: X: " << offset.X().value() << ", Y: " << offset.Y().value() << "." << std::endl;

  // Set the new pose to the swerve drive
  swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::AlignToTarget(int target, frc::Translation2d offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  // Get distance to target
  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);

  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  //frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);
  
  units::meter_t x_offset = distance.first * units::math::cos(distance.second);
  units::meter_t y_offset = distance.first * units::math::sin(distance.second);

  frc::Pose2d pose = frc::Pose2d(y_offset, x_offset, distance.second);
  

  // Print the results
  std::cout << "Aligning to tag " << target << ": X: " << pose.X().value() << " Y: " << pose.Y().value() << " Angle: " << pose.Rotation().Degrees().value() << ". At offset: X: " << offset.X().value() << ", Y: " << offset.Y().value() << "." << std::endl;

  // Set the new pose to the swerve drive
  //swerveDrive->SetPose(pose);

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
