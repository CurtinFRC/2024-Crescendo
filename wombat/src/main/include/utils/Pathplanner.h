#pragma once

#include "utils/Util.h"

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


namespace wom {
namespace utils {
    class Pathplanner {
    public:
        Pathplanner();

        frc::Trajectory getTrajectory(std::string_view path);

    private:
        fs::path deploy_directory = frc::filesystem::GetDeployDirectory();
    };
}
}
