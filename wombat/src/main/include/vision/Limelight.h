// Copyright (c) 2023 CurtinFRC
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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviour/HasBehaviour.h"
#include "utils/Util.h"

namespace wom {
namespace vision {
  class Limelight : public behaviour::HasBehaviour {
   public:
    explicit Limelight(std::string limelightName);

    std::string GetName();

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    std::pair<double, double> GetOffset();

    std::vector<double> GetAprilTagData(std::string dataName);

    void OnUpdate(units::time::second_t dt);
    void OnStart();

    bool IsAtSetPoseVision(frc::Pose3d pose, units::second_t dt);

    units::meters_per_second_t GetSpeed(frc::Pose3d pose1, frc::Pose3d pose2, units::second_t dt);

    frc::Pose3d GetPose();

   private:
    std::string _limelightName;
  };
}  // namespace vision
}  // namespace wom
