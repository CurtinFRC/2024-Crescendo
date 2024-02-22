// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "Wombat.h"
#include "behaviour/HasBehaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "units/length.h"
#include "units/time.h"
#include "vision/Vision.h"

enum class VisionType { kLimelight = 0, kGenericCamera = 1 };

class AlignToAprilTag : public behaviour::Behaviour {
 public:
  explicit AlignToAprilTag(Vision* vision, VisionTarget target, wom::SwerveDrive* swerveDrive,
                           units::meter_t offset = 0_m);

  void OnTick(units::second_t dt);

 private:
  Vision* _vision;
  wom::SwerveDrive* _swerveDrive;
  VisionTarget _target;
  units::meter_t _offset;
};

class TurnToAprilTag : public behaviour::Behaviour {
 public:
  explicit TurnToAprilTag(Vision* vision, VisionTarget target, wom::SwerveDrive* swerveDrive);

  void OnTick(units::second_t dt);

 private:
  Vision* _vision;
  wom::SwerveDrive* _swerveDrive;
  VisionTarget _target;
};

class LockOnToTarget : public behaviour::Behaviour {
 public:
  explicit LockOnToTarget(Vision* vision, VisionTargetObjects target, wom::SwerveDrive* swerveDrive);
  explicit LockOnToTarget(wom::PhotonVision* camera, Vision* limelight, wom::SwerveDrive* swerveDrive);

  void OnTick(units::second_t dt);

 private:
  Vision* _vision;
  wom::PhotonVision* _camera;

  VisionTargetObjects _target;
  wom::SwerveDrive* _swerveDrive;
  VisionType _type;
};
