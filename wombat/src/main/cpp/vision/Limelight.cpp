// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/Limelight.h"

#include <iostream>

#include "utils/Util.h"

wom::vision::Limelight::Limelight(std::string limelightName)
    : _limelightName(limelightName) {}

std::string wom::vision::Limelight::GetName() {
  return _limelightName;
}

std::pair<double, double> wom::vision::Limelight::GetOffset() {
  std::pair<double, double> offset;

  offset.first = table->GetNumber("tx", 0.0);
  offset.second = table->GetNumber("ty", 0.0);

  return offset;
}

std::vector<double> wom::vision::Limelight::GetAprilTagData(
    LimelightAprilTagData dataType) {
  std::string dataName;

  switch (dataType) {
    case LimelightAprilTagData::kBotpose:
      dataName = "botpose";
      break;

    case LimelightAprilTagData::kBotpose_wpiblue:
      dataName = "botpose_wpiblue";
      break;

    case LimelightAprilTagData::kBotpose_wpired:
      dataName = "botpose_wpired";
      break;

    case LimelightAprilTagData::kCamerapose_targetspace:
      dataName = "camerapose_targetspace";
      break;

    case LimelightAprilTagData::kTargetpose_cameraspace:
      dataName = "targetpose_cameraspace";
      break;

    case LimelightAprilTagData::kTargetpose_robotspace:
      dataName = "targetpose_robotspace";
      break;

    case LimelightAprilTagData::kBotpose_targetspace:
      dataName = "botpose_targetspace	";
      break;

    case LimelightAprilTagData::kCamerapose_robotspace:
      dataName = "camerapose_robotspace";
      break;

    case LimelightAprilTagData::kTid:
      dataName = "tid";
      break;
  }

  return table->GetNumberArray(dataName, std::vector<double>(6));
}

double wom::vision::Limelight::GetTargetingData(LimelightTargetingData dataType,
                                                double defaultValue) {
  std::string dataName;

  switch (dataType) {
    case LimelightTargetingData::kTv:
      dataName = "tv";
      break;

    case LimelightTargetingData::kTx:
      dataName = "tx";
      break;

    case LimelightTargetingData::kTy:
      dataName = "ty";
      break;

    case LimelightTargetingData::kTa:
      dataName = "ta";
      break;

    case LimelightTargetingData::kTl:
      dataName = "tl";
      break;

    case LimelightTargetingData::kCl:
      dataName = "cl";
      break;

    case LimelightTargetingData::kTshort:
      dataName = "tshort";
      break;

    case LimelightTargetingData::kTlong:
      dataName = "tlong";
      break;

    case LimelightTargetingData::kThor:
      dataName = "thor";
      break;

    case LimelightTargetingData::kTvert:
      dataName = "tvert";
      break;

    case LimelightTargetingData::kGetpipe:
      dataName = "getpipe";
      break;

    case LimelightTargetingData::kJson:
      dataName = "json";
      break;

    case LimelightTargetingData::kTclass:
      dataName = "tclass";
      break;

    case LimelightTargetingData::kTc:
      dataName = "tc";
      break;
  }

  return table->GetNumber(dataName, defaultValue);
}

void wom::vision::Limelight::SetLEDMode(LimelightLEDMode mode) {
  table->PutNumber("ledMode", static_cast<double>(mode));
}

void wom::vision::Limelight::SetCamMode(LimelightCamMode mode) {
  table->PutNumber("camMode", static_cast<double>(mode));
}

void wom::vision::Limelight::SetPipeline(LimelightPipeline pipeline) {
  table->PutNumber("pipeline", static_cast<double>(pipeline));
}

void wom::vision::Limelight::SetStreamMode(LimelightStreamMode mode) {
  table->PutNumber("stream", static_cast<double>(mode));
}

void wom::vision::Limelight::SetSnapshotMode(LimelightSnapshotMode mode) {
  table->PutNumber("snapshot", static_cast<double>(mode));
}

void wom::vision::Limelight::SetCrop(std::array<double, 4> crop) {
  table->PutNumberArray("camtran", crop);
}

units::meters_per_second_t wom::vision::Limelight::GetSpeed(
    frc::Pose3d pose1, frc::Pose3d pose2, units::second_t dt) {
  frc::Transform3d dPose{pose1, pose2};
  frc::Translation3d dTranslation = dPose.Translation();

  units::meter_t y = dTranslation.Y();
  units::meter_t x = dTranslation.X();
  units::radian_t theta = units::math::atan(y / x);
  units::meter_t dTRANSLATION = x / units::math::cos(theta);
  return units::math::fabs(dTRANSLATION / dt);
}

frc::Pose3d wom::vision::Limelight::GetPose() {
  std::vector<double> pose = GetAprilTagData(LimelightAprilTagData::kBotpose);
  return frc::Pose3d(
      pose[1] * 1_m, 1_m * pose[2], 1_m * pose[3],
      frc::Rotation3d(1_deg * (pose[4]), 1_deg * (pose[5]), 1_deg * (pose[6])));
}

void wom::vision::Limelight::OnStart() {
  std::cout << "starting" << std::endl;
}

void wom::vision::Limelight::OnUpdate(units::time::second_t dt) {
  wom::utils::WritePose3NT(table, GetPose());
}

bool wom::vision::Limelight::IsAtSetPoseVision(frc::Pose3d pose,
                                               units::second_t dt) {
  frc::Pose3d actualPose = GetPose();
  frc::Pose3d relativePose = actualPose.RelativeTo(pose);
  return (units::math::fabs(relativePose.X()) < 0.01_m &&
          units::math::fabs(relativePose.Y()) < 0.01_m &&
          GetSpeed(pose, GetPose(), dt) < 1_m / 1_s);
}
