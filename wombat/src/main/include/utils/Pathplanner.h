// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

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
}  // namespace utils
}  // namespace wom
