// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "vision/VisionBehaviours.h"

#include "frc/geometry/Pose2d.h"
#include "units/angle.h"

AlignToAprilTag::AlignToAprilTag(Vision* vision, VisionTarget target, wom::SwerveDrive* swerveDrive)
    : _vision(vision), _target(target), _swerveDrive(swerveDrive) {}

void AlignToAprilTag::OnTick(units::second_t dt) {
  frc::Pose2d pose = _vision->TurnToTarget(_target, _swerveDrive);

  if (_vision->IsAtPose(pose, dt)) {
    SetDone();
  }
}
