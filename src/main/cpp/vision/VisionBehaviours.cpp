// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/VisionBehaviours.h"

#include "frc/geometry/Pose2d.h"
#include "units/angle.h"
#include "vision/Vision.h"

AlignToAprilTag::AlignToAprilTag(Vision* vision, VisionTarget target, wom::SwerveDrive* swerveDrive,
                                 units::meter_t offset)
    : _vision(vision), _swerveDrive(swerveDrive), _target(target), _offset(offset) {}

void AlignToAprilTag::OnTick(units::second_t dt) {
  frc::Pose2d pose = _vision->AlignToTarget(_target, _offset, _swerveDrive);

  if (_vision->IsAtPose(pose, dt)) {
    SetDone();
  }
}

TurnToAprilTag::TurnToAprilTag(Vision* vision, VisionTarget target, wom::SwerveDrive* swerveDrive)
    : _vision(vision), _swerveDrive(swerveDrive), _target(target) {}

void TurnToAprilTag::OnTick(units::second_t dt) {
  frc::Pose2d pose = _vision->TurnToTarget(_target, _swerveDrive);

  if (_vision->IsAtPose(pose, dt)) {
    SetDone();
  }
}

LockOnToTarget::LockOnToTarget(Vision* vision, VisionTargetObjects target, wom::SwerveDrive* swerveDrive)
    : _vision(vision),
      _target(target),
      _camera(nullptr),
      _swerveDrive(swerveDrive),
      _type(VisionType::kLimelight) {}

LockOnToTarget::LockOnToTarget(wom::PhotonVision* vision, Vision* limelight, wom::SwerveDrive* swerveDrive)
    : _camera(vision), _swerveDrive(swerveDrive), _vision(limelight), _type(VisionType::kGenericCamera) {}

void LockOnToTarget::OnTick(units::second_t dt) {
  if (!_vision->TargetIsVisible(VisionTargetObjects::kNote)) {
    SetDone();
  }

  units::degree_t angle;

  switch (_type) {
    case VisionType::kLimelight: {
      angle = _vision->LockOn(_target);
    }
    case VisionType::kGenericCamera: {
      angle = units::degree_t{_camera->GetTargetPitch(_camera->GetTarget())};
    }
  }

  frc::Pose2d pose = frc::Pose2d(_swerveDrive->GetPose().X(), _swerveDrive->GetPose().Y(), angle);

  _swerveDrive->SetPose(pose);

  if (_vision->IsAtPose(pose, dt)) {
    SetDone();
  }
}
