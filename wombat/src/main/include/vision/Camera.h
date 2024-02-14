// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <photonlib/PhotonCamera.h>

#include <string>
#include <utility>
#include <vector>

#include "frc/geometry/Transform2d.h"
#include "frc/geometry/Transform3d.h"
#include "photonlib/PhotonPipelineResult.h"
#include "photonlib/PhotonTrackedTarget.h"
#include "wpi/SmallVector.h"

namespace wom {
namespace vision {

enum class PhotonVisionModes { kNormal = 0, kObject = 1 };

using photonTarget = photonlib::PhotonTrackedTarget;

using PhotonVisionLEDMode = photonlib::LEDMode;

class PhotonVision {
 public:
  PhotonVision();
  explicit PhotonVision(std::string name);

  photonlib::PhotonPipelineResult GetResult();

  void SetLED(PhotonVisionLEDMode mode);
  void SetPipelineIndex(int index);

  bool HasTarget();
  std::vector<photonTarget> GetTargets();
  photonTarget GetTarget();

  double GetTargetYaw(photonTarget target);
  double GetTargetPitch(photonTarget target);
  double GetTargetArea(photonTarget target);
  double GetTargetSkew(photonTarget target);

  frc::Transform3d GetCameraToTarget(photonTarget target);
  std::vector<std::pair<double, double>> GetTargetCorners(photonTarget target);

  int GetTargetId(photonTarget target);
  frc::Transform3d BestCameraToTarget(photonTarget target);
  frc::Transform3d AlternateCameraToTarget(photonTarget target);

  PhotonVisionModes GetMode();
  void SetMode(PhotonVisionModes mode);

 private:
  void Initialize();

  std::string _name;

  PhotonVisionModes _mode = PhotonVisionModes::kObject;

  photonlib::PhotonCamera* _camera;
};
}  // namespace vision
}  // namespace wom
