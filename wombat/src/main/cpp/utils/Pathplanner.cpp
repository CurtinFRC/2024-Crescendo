// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/Pathplanner.h"

using namespace wom;

utils::Pathplanner::Pathplanner() {}

frc::Trajectory utils::Pathplanner::getTrajectory(std::string_view path) {
  try {
    fs::path path_location = deploy_directory / path;
    return frc::TrajectoryUtil::FromPathweaverJson(path_location.string());
  } catch (std::exception& e) {
    return getTrajectory(path);
  }
}
