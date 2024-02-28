// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/Filesystem.h>
#include <units/length.h>
#include <units/math.h>
#include <wpi/fs.h>
#include <wpi/json.h>

#include <string>
#include <utility>
#include <vector>

#include "Wombat.h"
#include "units/angle.h"

#define APRILTAGS_MAX 16
#define APRILTAGS_MIN 0

enum class VisionTarget {
  /*
      Left is toward the blue side of the diagram here:
     https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf

      The numbers are the numbers on the field diagram (and the numbers on the tags).
  */
  kBlueAmp = 6,
  kBlueSpeakerCenter = 7,
  kBlueSpeakerOffset = 8,
  kBlueChain1 = 15,
  kBlueChain2 = 16,
  kBlueChain3 = 14,
  kBlueSourceLeft = 1,
  kBlueSourceRight = 2,

  kRedAmp = 5,
  kRedSpeakerCenter = 4,
  kRedSpeakerOffset = 3,

  kRedChain1 = 12,
  kRedChain2 = 11,
  kRedChain3 = 13,
  kRedSourceLeft = 9,
  kRedSourceRight = 10
};

enum class VisionModes { kAprilTag = 0, kRing = 1 };

enum class VisionTargetObjects { kNote };

struct AprilTag {
  int id;
  double size;
  std::array<std::array<double, 4>, 4> transform;
  bool unique;

  frc::Pose3d pos;
  units::radian_t yaw;
  units::radian_t pitch;
  units::radian_t roll;
};

class FMAP {
 public:
  explicit FMAP(std::string path);

  std::vector<AprilTag> GetTags();

 private:
  std::vector<AprilTag> _tags;
  std::string _path;
  std::string deploy_dir;
};

class Vision {
 public:
  Vision(std::string name, FMAP fmap);

  std::pair<units::meter_t, units::degree_t> GetDistanceToTarget(VisionTarget target);
  std::pair<units::meter_t, units::degree_t> GetDistanceToTarget(int id);

  void SetMode(VisionModes mode);

  frc::Pose3d GetPose();

  frc::Pose2d AlignToTarget(VisionTarget target, units::meter_t offset, wom::SwerveDrive* swerveDrive);
  frc::Pose2d AlignToTarget(int target, units::meter_t offset, wom::SwerveDrive* swerveDrive);

  frc::Pose2d AlignToTarget(VisionTarget target, frc::Translation2d offset, wom::SwerveDrive* swerveDrive);
  frc::Pose2d AlignToTarget(int target, frc::Translation2d offset, wom::SwerveDrive* swerveDrive);

  frc::Pose2d TurnToTarget(int target, wom::SwerveDrive* swerveDrive);
  frc::Pose2d TurnToTarget(VisionTarget target, wom::SwerveDrive* swerveDrive);

  std::pair<frc::Pose2d, units::degree_t> GetAngleToObject(VisionTargetObjects object);
  units::degree_t LockOn(VisionTargetObjects target);

  std::vector<AprilTag> GetTags();

  bool IsAtPose(frc::Pose3d pose, units::second_t dt);
  bool IsAtPose(frc::Pose2d pose, units::second_t dt);

  bool TargetIsVisible(VisionTargetObjects target);

  int CurrentAprilTag();

  wom::Limelight* GetLimelight();

 private:
  wom::Limelight* _limelight;
  FMAP _fmap;
};
