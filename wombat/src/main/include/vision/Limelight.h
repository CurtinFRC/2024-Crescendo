// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/json.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace wom {
namespace vision {

enum class LimelightLEDMode { kPipelineDefault = 0, kForceOff = 1, kForceBlink = 2, kForceOn = 3 };

enum class LimelightCamMode { kVisionProcessor = 0, kDriverCamera = 1 };

enum class LimelightStreamMode { kStandard = 0, kPiPMain = 1, kPiPSecondary = 2 };

enum class LimelightSnapshotMode { kReset = 0, kSingle = 1 };

enum class LimelightPipeline {
  kPipeline0 = 0,
  kPipeline1 = 1,
  kPipeline2 = 2,
  kPipeline3 = 3,
  kPipeline4 = 4,
  kPipeline5 = 5,
  kPipeline6 = 6,
  kPipeline7 = 7,
  kPipeline8 = 8,
  kPipeline9 = 9
};

enum class LimelightTargetingData {
  kTv = 0,
  kTx = 1,
  kTy = 2,
  kTa = 3,
  kTl = 4,
  kCl = 5,
  kTshort = 6,
  kTlong = 7,
  kThor = 8,
  kTvert = 9,
  kGetpipe = 10,
  kJson = 11,
  kTclass = 12,
  kTc = 13,
  kTid = 14
};

enum class LimelightAprilTagData {
  kBotpose = 0,
  kBotpose_wpiblue = 1,
  kBotpose_wpired = 2,
  kCamerapose_targetspace = 3,
  kTargetpose_cameraspace = 4,
  kTargetpose_robotspace = 5,
  kBotpose_targetspace = 6,
  kCamerapose_robotspace = 7,
};

class Limelight {
 public:
  explicit Limelight(std::string limelightName);

  std::string GetName();

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  std::pair<double, double> GetOffset();

  std::vector<double> GetAprilTagData(LimelightAprilTagData dataType);
  double GetTargetingData(LimelightTargetingData dataType, double defaultValue = 0.0);
  void SetLEDMode(LimelightLEDMode mode);
  void SetCamMode(LimelightCamMode mode);
  void SetPipeline(LimelightPipeline pipeline);
  void SetStreamMode(LimelightStreamMode mode);
  void SetSnapshotMode(LimelightSnapshotMode mode);
  void SetCrop(std::array<double, 4> crop);

  void OnUpdate(units::time::second_t dt);
  void OnStart();

  bool IsAtSetPoseVision(frc::Pose3d pose, units::second_t dt);
  bool IsAtSetPoseVision(frc::Pose2d pose, units::second_t dt);

  units::meters_per_second_t GetSpeed(frc::Pose3d pose1, frc::Pose3d pose2, units::second_t dt);
  units::meters_per_second_t GetSpeed(frc::Pose2d pose1, frc::Pose2d pose2, units::second_t dt);

  frc::Pose3d GetPose();

  bool HasTarget();

 private:
  std::string _limelightName;
};
}  // namespace vision
}  // namespace wom
