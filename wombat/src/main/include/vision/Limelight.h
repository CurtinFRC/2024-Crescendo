#pragma once

#include "behaviour/HasBehaviour.h"
#include "utils/Util.h"

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <utility>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/json.h>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Pose2d.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

namespace wom {
namespace vision {
  class Limelight : public behaviour::HasBehaviour{
   public:
    Limelight(std::string *limelightName);

    std::string *GetName();

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    std::pair<double, double> GetOffset();
    
    std::vector<double> GetAprilTagData(std::string dataName);

    void OnUpdate(units::time::second_t dt);
    void OnStart();

    bool IsAtSetPoseVision(frc::Pose3d pose, units::second_t dt);

    units::meters_per_second_t GetSpeed(frc::Pose3d pose1, frc::Pose3d pose2, units::second_t dt);

    frc::Pose3d GetPose();

   private:
    std::string *_limelightName;
  };
}
}
