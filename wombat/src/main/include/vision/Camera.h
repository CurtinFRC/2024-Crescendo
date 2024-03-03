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
#include "units/angle.h"
#include "wpi/SmallVector.h"

namespace wom {
namespace vision {

enum class PhotonVisionModes { kNormal = 0, kNotes = 1 };

using PhotonTarget = photonlib::PhotonTrackedTarget;

using PhotonVisionLEDMode = photonlib::LEDMode;

class PhotonVision {
 public:
  PhotonVision();
  explicit PhotonVision(std::string name);

  photonlib::PhotonPipelineResult GetResult();

  void SetLED(PhotonVisionLEDMode mode);
  void SetPipelineIndex(int index);

  bool HasTarget();
  std::vector<PhotonTarget> GetTargets();
  PhotonTarget GetTarget();

  double GetTargetYaw(PhotonTarget target);
  double GetTargetPitch(PhotonTarget target);
  double GetTargetArea(PhotonTarget target);
  double GetTargetSkew(PhotonTarget target);

  frc::Transform3d GetCameraToTarget(PhotonTarget target);
  std::vector<std::pair<double, double>> GetTargetCorners(PhotonTarget target);

  int GetTargetId(PhotonTarget target);
  frc::Transform3d BestCameraToTarget(PhotonTarget target);
  frc::Transform3d AlternateCameraToTarget(PhotonTarget target);

  PhotonVisionModes GetMode();
  void SetMode(PhotonVisionModes mode);

 private:
  void Initialize();

  std::string _name;

  PhotonVisionModes _mode = PhotonVisionModes::kNotes;

  photonlib::PhotonCamera* _camera;
};
}  // namespace vision
}  // namespace wom
