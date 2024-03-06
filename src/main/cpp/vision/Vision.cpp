// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/Vision.h"

#include <utility>
#include <variant>

#include "units/length.h"
#include "units/math.h"
#include "vision/Limelight.h"

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

    tag.pos = frc::Pose3d(units::meter_t{tag.transform[0][3]}, units::meter_t{tag.transform[1][3]},
                          units::meter_t{tag.transform[2][3]}, frc::Rotation3d(0_deg, 0_deg, 0_deg));

    _tags.push_back(tag);
  }

  file.close();

  std::cout << "Loaded " << _tags.size() << " tags" << std::endl;

  for (int i = 0; i < _tags.size(); i++) {
    std::cout << "Tag " << _tags[i].id << " is at: X: " << _tags[i].pos.X().value()
              << " Y: " << _tags[i].pos.Y().value() << " Z: " << _tags[i].pos.Z().value() << std::endl;
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
      // frc::Pose3d pose = _limelight->GetPose();
      // frc::Pose3d pose = frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0_deg, 0_deg, 0_deg));

      SetMode(VisionModes::kAprilTag);

      // Get distance to target
      // Get current position from Limelight
      frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
      // frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);

      units::meter_t a = tag.pos.X() - current_pose.X();
      units::meter_t b = tag.pos.Y() - current_pose.Y();

      units::radian_t theta = units::math::atan2(b, a);

      // std::cout << "A: " << a.value() << ", B:" << b.value() << std::endl;
      units::meter_t r = units::meter_t{std::sqrt(std::pow(a.value(), 2) + std::pow(b.value(), 2))};

      // units::meter_t x = current_pose.X() + r * units::math::cos(theta);
      // units::meter_t y = current_pose.Y() + r * units::math::sin(theta);

      return std::make_pair(r, theta);
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
      // frc::Pose3d pose = _limelight->GetPose();
      // frc::Pose3d pose = frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0_deg, 0_deg, 0_deg));

      SetMode(VisionModes::kAprilTag);

      // Get distance to target
      // Get current position from Limelight
      frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
      // frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);

      units::meter_t a = tag.pos.X() - current_pose.X();
      units::meter_t b = tag.pos.Y() - current_pose.Y();

      units::radian_t theta = units::math::atan2(b, a);

      // std::cout << "A: " << a.value() << ", B:" << b.value() << std::endl;
      units::meter_t r = units::meter_t{std::sqrt(std::pow(a.value(), 2) + std::pow(b.value(), 2))};

      units::meter_t x = current_pose.X() + r * units::math::cos(theta);
      units::meter_t y = current_pose.Y() + r * units::math::sin(theta);

      return std::make_pair(r, theta);
    }
  }

  return std::make_pair(0_m, 0_deg);
}

std::vector<AprilTag> Vision::GetTags() {
  return _fmap.GetTags();
}

frc::Pose2d Vision::AlignToTarget(VisionTarget target, units::meter_t offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  // Get distance to target
  AprilTag tag = GetTags()[static_cast<int>(target) - 1];
  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  // frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);

  units::meter_t a = tag.pos.X() - current_pose.X();
  units::meter_t b = tag.pos.Y() - current_pose.Y();

  units::radian_t theta = units::math::atan2(b, a);

  // std::cout << "A: " << a.value() << ", B:" << b.value() << std::endl;
  units::meter_t r = units::meter_t{std::sqrt(std::pow(a.value(), 2) + std::pow(b.value(), 2))};

  units::meter_t x = current_pose.X() + r * units::math::cos(theta);
  units::meter_t y = current_pose.Y() + r * units::math::sin(theta);

  // reduce both offsets by the offset parameter (in relative amount)
  x += offset * units::math::cos(theta);
  y += offset * units::math::sin(theta);

  frc::Pose2d pose = frc::Pose2d(x, y, theta);

  // Print the results
  // std::cout << "Aligning to tag " << target << ": X: " << pose.X().value() << " Y: " << pose.Y().value() <<
  // " Angle: " << pose.Rotation().Degrees().value() << ". At offset " << offset.value() << "." << std::endl;

  // Set the new pose to the swerve drive
  // swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::AlignToTarget(int target, units::meter_t offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  // Get distance to target
  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);
  AprilTag tag = GetTags()[target - 1];
  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  // frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);

  units::meter_t a = tag.pos.X() - current_pose.X();
  units::meter_t b = tag.pos.Y() - current_pose.Y();

  units::radian_t theta = units::math::atan2(b, a);

  // std::cout << "A: " << a.value() << ", B:" << b.value() << std::endl;
  units::meter_t r = units::meter_t{std::sqrt(std::pow(a.value(), 2) + std::pow(b.value(), 2))};

  units::meter_t x = current_pose.X() + r * units::math::cos(theta);
  units::meter_t y = current_pose.Y() + r * units::math::sin(theta);

  // reduce both offsets by the offset parameter (in relative amount)
  x += offset * units::math::cos(theta);
  y += offset * units::math::sin(theta);

  frc::Pose2d pose = frc::Pose2d(x, y, theta);

  // Print the results
  // std::cout << "Aligning to tag " << target << ": X: " << pose.X().value() << " Y: " << pose.Y().value() <<
  // " Angle: " << pose.Rotation().Degrees().value() << ". At offset " << offset.value() << "." << std::endl;

  // Set the new pose to the swerve drive
  // swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::AlignToTarget(VisionTarget target, frc::Translation2d offset,
                                  wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  // Get distance to target
  AprilTag tag = GetTags()[static_cast<int>(target) - 1];
  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  // frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);

  units::meter_t a = tag.pos.X() - current_pose.X();
  units::meter_t b = tag.pos.Y() - current_pose.Y();

  units::radian_t theta = units::math::atan2(b, a);

  // std::cout << "A: " << a.value() << ", B:" << b.value() << std::endl;
  units::meter_t r = units::meter_t{std::sqrt(std::pow(a.value(), 2) + std::pow(b.value(), 2))};

  units::meter_t x = current_pose.X() + r * units::math::cos(theta);
  units::meter_t y = current_pose.Y() + r * units::math::sin(theta);

  // reduce both offsets by the offset parameter (in relative amount)
  x += offset.X();
  y += offset.Y();

  frc::Pose2d pose = frc::Pose2d(x, y, theta);

  // Print the results
  // std::cout << "Aligning to tag " << target << ": X: " << pose.X().value() << " Y: " << pose.Y().value() <<
  // " Angle: " << pose.Rotation().Degrees().value() << ". At offset " << offset.value() << "." << std::endl;

  // Set the new pose to the swerve drive
  // swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::AlignToTarget(int target, frc::Translation2d offset, wom::SwerveDrive* swerveDrive) {
  SetMode(VisionModes::kAprilTag);

  // Get distance to target
  std::pair<units::meter_t, units::degree_t> distance = GetDistanceToTarget(target);
  AprilTag tag = GetTags()[target - 1];
  // Get current position from Limelight
  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  // frc::Pose2d current_pose = frc::Pose2d(0_m, 0_m, 0_deg);

  units::meter_t a = tag.pos.X() - current_pose.X();
  units::meter_t b = tag.pos.Y() - current_pose.Y();

  units::radian_t theta = units::math::atan2(b, a);

  // std::cout << "A: " << a.value() << ", B:" << b.value() << std::endl;
  units::meter_t r = units::meter_t{std::sqrt(std::pow(a.value(), 2) + std::pow(b.value(), 2))};

  units::meter_t x = current_pose.X() + r * units::math::cos(theta);
  units::meter_t y = current_pose.Y() + r * units::math::sin(theta);

  // reduce both offsets by the offset parameter (in relative amount)
  x += offset.X();
  y += offset.Y();

  frc::Pose2d pose = frc::Pose2d(x, y, theta);

  // Print the results
  // std::cout << "Aligning to tag " << target << ": X: " << pose.X().value() << " Y: " << pose.Y().value() <<
  // " Angle: " << pose.Rotation().Degrees().value() << ". At offset " << offset.value() << "." << std::endl;

  // Set the new pose to the swerve drive
  // swerveDrive->SetPose(pose);

  return pose;
}

frc::Pose2d Vision::TurnToTarget(int target, wom::SwerveDrive* swerveDrive) {
  AprilTag tag = GetTags()[target];

  units::degree_t angle = GetDistanceToTarget(target).second;

  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();
  // frc::Pose2d current_pose = frc::Pose2d();

  frc::Pose2d pose = frc::Pose2d(current_pose.X(), current_pose.Y(), angle);

  // std::cout << pose.Rotation().Degrees().value() << std::endl;

  swerveDrive->TurnToAngle(angle);

  return pose;
}

frc::Pose2d Vision::TurnToTarget(VisionTarget target, wom::SwerveDrive* swerveDrive) {
  AprilTag tag = GetTags()[static_cast<int>(target)];

  units::degree_t angle = GetDistanceToTarget(target).second;

  frc::Pose2d current_pose = _limelight->GetPose().ToPose2d();

  // frc::Pose2d current_pose = swerveDrive->GetPose();

  frc::Pose2d pose = frc::Pose2d(current_pose.X(), current_pose.Y(), angle);

  std::cout << pose.Rotation().Degrees().value() << std::endl;

  swerveDrive->TurnToAngle(angle);

  return pose;
}

std::pair<frc::Pose2d, units::degree_t> Vision::GetAngleToObject(VisionTargetObjects object) {
  SetMode(VisionModes::kRing);

  if (TargetIsVisible(object)) {
    frc::Pose2d pose = _limelight->GetPose().ToPose2d();

    units::degree_t offset = units::degree_t{
        _limelight->GetOffset()
            .first};  // degrees are how much the robot has to turn for the target to be in the center

    pose.RotateBy(offset);

    return std::make_pair(pose, offset);
  } else {
    return std::make_pair(frc::Pose2d(), 0_deg);
  }
}

units::degree_t Vision::LockOn(VisionTargetObjects object) {
  SetMode(VisionModes::kRing);

  units::degree_t angle;

  if (TargetIsVisible(object)) {
    angle = GetAngleToObject(object).second;
  }

  return angle;
}

bool Vision::IsAtPose(frc::Pose3d pose, units::second_t dt) {
  return _limelight->IsAtSetPoseVision(pose, dt);
}

bool Vision::IsAtPose(frc::Pose2d pose, units::second_t dt) {
  return _limelight->IsAtSetPoseVision(pose, dt);
}

void Vision::SetMode(VisionModes mode) {
  if (static_cast<int>(mode) == _limelight->GetTargetingData(wom::LimelightTargetingData::kGetpipe)) {
    return;
  }

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

wom::Limelight* Vision::GetLimelight() {
  return _limelight;
}
