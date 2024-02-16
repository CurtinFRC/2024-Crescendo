// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/Camera.h"

#include <cstddef>
#include <vector>

#include "frc/geometry/Transform3d.h"

namespace wom {
namespace vision {
PhotonVision::PhotonVision() : _name("photonvision") {
  Initialize();
}

PhotonVision::PhotonVision(std::string name) : _name(name) {
  Initialize();
}

void PhotonVision::Initialize() {
  _camera = new photonlib::PhotonCamera(_name);

  SetMode(_mode);
}

photonlib::PhotonPipelineResult PhotonVision::GetResult() {
  return _camera->GetLatestResult();
}

void PhotonVision::SetLED(PhotonVisionLEDMode mode) {
  _camera->SetLEDMode(mode);
}

void PhotonVision::SetPipelineIndex(int index) {
  _camera->SetPipelineIndex(index);
}

bool PhotonVision::HasTarget() {
  return GetResult().HasTargets();
}

std::vector<PhotonTarget> PhotonVision::GetTargets() {
  const std::span<const PhotonTarget> res = GetResult().GetTargets();

  std::vector<PhotonTarget> targets;

  for (size_t i = 0; i < res.size(); i++) {
    targets.push_back(res[i]);
  }

  return targets;
}

PhotonTarget PhotonVision::GetTarget() {
  return GetResult().GetBestTarget();
}

double PhotonVision::GetTargetYaw(PhotonTarget target) {
  return target.GetYaw();
}

double PhotonVision::GetTargetPitch(PhotonTarget target) {
  return target.GetPitch();
}

double PhotonVision::GetTargetArea(PhotonTarget target) {
  return target.GetArea();
}

double PhotonVision::GetTargetSkew(PhotonTarget target) {
  return target.GetSkew();
}

frc::Transform3d PhotonVision::GetCameraToTarget(PhotonTarget target) {
  return target.GetBestCameraToTarget();
}

std::vector<std::pair<double, double>> PhotonVision::GetTargetCorners(PhotonTarget target) {
  return target.GetDetectedCorners();
}

int PhotonVision::GetTargetId(PhotonTarget target) {
  return target.GetFiducialId();
}

frc::Transform3d PhotonVision::BestCameraToTarget(PhotonTarget target) {
  return target.GetBestCameraToTarget();
}

frc::Transform3d PhotonVision::AlternateCameraToTarget(PhotonTarget target) {
  return target.GetAlternateCameraToTarget();
}

PhotonVisionModes PhotonVision::GetMode() {
  return _mode;
}

void PhotonVision::SetMode(PhotonVisionModes mode) {
  _mode = mode;

  SetPipelineIndex(static_cast<int>(mode));
}

}  // namespace vision
}  // namespace wom
