// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/Camera.h"

#include <cstddef>
#include <vector>

#include "frc/geometry/Transform3d.h"

PhotonVision::PhotonVision() : _name("photonvision") {
  Initialize();
};

PhotonVision::PhotonVision(std::string name) : _name(name) {
  Initialize();
};

void PhotonVision::Initialize() {
  _camera = new photonlib::PhotonCamera(_name);

  SetMode(PhotonVisionModes::kObject);
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

std::vector<photonTarget> PhotonVision::GetTargets() {
  const std::span<const photonTarget> res = GetResult().GetTargets();

  std::vector<photonTarget> targets;

  for (size_t i = 0; i < res.size(); i++) {
    targets.push_back(res[i]);
  };

  return targets;
}

photonTarget PhotonVision::GetTarget() {
  return GetResult().GetBestTarget();
}

double PhotonVision::GetTargetYaw(photonTarget target) {
  return target.GetYaw();
}

double PhotonVision::GetTargetPitch(photonTarget target) {
  return target.GetPitch();
}

double PhotonVision::GetTargetArea(photonTarget target) {
  return target.GetArea();
}

double PhotonVision::GetTargetSkew(photonTarget target) {
  return target.GetSkew();
}

frc::Transform3d PhotonVision::GetCameraToTarget(photonTarget target) {
  return target.GetBestCameraToTarget();
}

std::vector<std::pair<double, double>> PhotonVision::GetTargetCorners(photonTarget target) {
  return target.GetDetectedCorners();
}

int PhotonVision::GetTargetId(photonTarget target) {
  return target.GetFiducialId();
}

frc::Transform3d PhotonVision::BestCameraToTarget(photonTarget target) {
  return target.GetBestCameraToTarget();
}

frc::Transform3d PhotonVision::AlternateCameraToTarget(photonTarget target) {
  return target.GetAlternateCameraToTarget();
}

PhotonVisionModes PhotonVision::GetMode() {
  return _mode;
}

void PhotonVision::SetMode(PhotonVisionModes mode) {
  _mode = mode;

  SetPipelineIndex(static_cast<int>(mode));
}
