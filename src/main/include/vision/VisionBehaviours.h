// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "Wombat.h"
#include "behaviour/HasBehaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "units/time.h"
#include "vision/Vision.h"

class AlignToAprilTag : public behaviour::Behaviour {
 public:
  explicit AlignToAprilTag(Vision* vision, VisionTarget target, wom::SwerveDrive* swerveDrive);

  void OnTick(units::second_t dt);

 private:
  Vision* _vision;
  wom::SwerveDrive* _swerveDrive;
  VisionTarget _target;
};
