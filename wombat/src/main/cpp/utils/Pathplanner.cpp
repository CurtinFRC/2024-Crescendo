// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/Pathplanner.h"

#include <frc/DriverStation.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "drivetrain/behaviours/SwerveBehaviours.h"
#include "frc/Filesystem.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/path/PathPoint.h"
#include "pathplanner/lib/path/RotationTarget.h"
#include "units/time.h"
#include "utils/Util.h"
#include "wpi/detail/conversions/to_json.h"
#include "wpi/json_fwd.h"

using namespace wom;

wom::utils::BezierPoint::BezierPoint(double anchorX, double anchorY, wpi::json prevControlX,
                                     wpi::json prevControlY, wpi::json nextControlX, wpi::json nextControlY,
                                     bool isLocked, wpi::json linkedName)
    : anchor({anchorX, anchorY}),
      prevControl{prevControlX.is_null() ? NAN : prevControlX.get<double>(),
                  prevControlY.is_null() ? NAN : prevControlY.get<double>()},
      nextControl{nextControlX.is_null() ? NAN : nextControlX.get<double>(),
                  nextControlY.is_null() ? NAN : nextControlY.get<double>()},
      isLocked(isLocked),
      linkedName(linkedName.is_null() ? "" : linkedName.get<std::string>()) {}

double wom::utils::BezierPoint::calculateAngle() const {
  return std::atan2(anchor.y - prevControl.y, anchor.x - prevControl.x);
}

double wom::utils::distance(const wom::utils::BezierPoint& p1, const wom::utils::BezierPoint& p2) {
  return std::sqrt(std::pow(p2.anchor.x - p1.anchor.x, 2) + std::pow(p2.anchor.y - p1.anchor.y, 2));
}

std::vector<wom::utils::BezierPoint> wom::utils::interpolateAtIntervals(const wom::utils::BezierPoint& p1,
                                                                        const wom::utils::BezierPoint& p2,
                                                                        double interval) {
  std::vector<wom::utils::BezierPoint> interpolatedPoints;

  double totalDistance = distance(p1, p2);
  int numPoints = static_cast<int>(std::ceil(totalDistance / interval));

  for (int i = 1; i <= numPoints; ++i) {
    double t = static_cast<double>(i) / (numPoints + 1);
    interpolatedPoints.emplace_back(
        p1.anchor.x + t * (p2.anchor.x - p1.anchor.x), p1.anchor.y + t * (p2.anchor.y - p1.anchor.y),
        p1.prevControl.x + t * (p2.prevControl.x - p1.prevControl.x),
        p1.prevControl.y + t * (p2.prevControl.y - p1.prevControl.y),
        p1.nextControl.x + t * (p2.nextControl.x - p1.nextControl.x),
        p1.nextControl.y + t * (p2.nextControl.y - p1.nextControl.y), p1.isLocked, p1.linkedName);
  }

  return interpolatedPoints;
}

std::vector<wom::utils::BezierPoint> wom::utils::createBezierPointsFromJson(const wpi::json& jsonData) {
  std::vector<wom::utils::BezierPoint> bezierPoints;

  for (const auto& point : jsonData["waypoints"]) {
    auto anchorX = point["anchor"].is_null() ? NAN : point["anchor"]["x"].get<double>();
    auto anchorY = point["anchor"].is_null() ? NAN : point["anchor"]["y"].get<double>();
    auto prevControlX = point["prevControl"].is_null() ? NAN : point["prevControl"]["x"].get<double>();
    auto prevControlY = point["prevControl"].is_null() ? NAN : point["prevControl"]["y"].get<double>();
    auto nextControlX = point["nextControl"].is_null() ? NAN : point["nextControl"]["x"].get<double>();
    auto nextControlY = point["nextControl"].is_null() ? NAN : point["nextControl"]["y"].get<double>();
    auto isLocked = point["isLocked"].is_null() ? false : point["isLocked"].get<bool>();
    auto linkedName = point["linkedName"].is_null() ? "" : point["linkedName"].get<std::string>();

    bezierPoints.emplace_back(anchorX, anchorY, prevControlX, prevControlY, nextControlX, nextControlY,
                              isLocked, linkedName);
  }

  return bezierPoints;
}

std::vector<wom::utils::BezierPoint> wom::utils::interpolateBezierPoints(
    const std::vector<wom::utils::BezierPoint>& bezierPoints, double interval) {
  std::vector<wom::utils::BezierPoint> interpolatedPoints;

  for (size_t i = 0; i < bezierPoints.size() - 1; ++i) {
    auto points = interpolateAtIntervals(bezierPoints[i], bezierPoints[i + 1], interval);
    interpolatedPoints.insert(interpolatedPoints.end(), points.begin(), points.end());
  }

  return interpolatedPoints;
}

// Command implementation
// template <typename Ret, typename... Args>
// utils::Command<Ret, Args...>::Command(std::string name, FunctionType func)
// : name_(std::move(name)), function_(std::move(func)), argumentTypes_{typeid(Args)...} {}

// template <typename Ret, typename... Args>
// Ret utils::Command<Ret, Args...>::Execute(Args... args) const {
// return function_(std::forward<Args>(args)...);
// }

// template <typename Ret, typename... Args>
// std::string utils::Command<Ret, Args...>::GetName() const {
// return name_;
// }

// template <typename Ret, typename... Args>
// bool utils::Command<Ret, Args...>::IsValid(std::vector<std::type_index> argTypes) const {
// return argumentTypes_ == argTypes;
// }

// Commands implementation
// template <typename... CommandTypes>
// template <typename... CommandArgs>
// utils::Commands<CommandTypes...>::Commands(CommandArgs&&... commands)
//     :
//     commands_{std::make_shared<utils::ConcreteCommand<CommandTypes>>(std::forward<CommandArgs>(commands))...}
//     {}
//
// template <typename... CommandTypes>
// template <typename Ret, typename... Args>
// Ret utils::Commands<CommandTypes...>::Execute(std::string command, Args&&... args) {
//     auto it = std::find_if(commands_.begin(), commands_.end(), [&command](const auto& cmd) {
//         return cmd->GetName() == command;
//     });
//
//     if (it != commands_.end()) {
//         return (*it)->template Execute<Ret>({std::forward<Args>(args)...});
//     } else {
//         throw std::invalid_argument("Command not found");
//     }
// }
//
// template <typename... CommandTypes>
// bool utils::Commands<CommandTypes...>::IsValid(std::string command, std::vector<std::type_index> argTypes)
// const {
//     auto it = std::find_if(commands_.begin(), commands_.end(), [&command](const auto& cmd) {
//         return cmd->GetName() == command;
//     });
//
//     if (it != commands_.end()) {
//         return (*it)->IsValid(argTypes);
//     } else {
//         throw std::invalid_argument("Command not found");
//     }
// }
//
// template <typename... CommandTypes>
// template <typename Ret, typename... Args>
// Ret utils::Commands<CommandTypes...>::Run(std::string commandString, Args&&... args) {
//     size_t pos = commandString.find("(");
//     if (pos != std::string::npos) {
//         std::string command = commandString.substr(0, pos);
//         std::string argString = commandString.substr(pos + 1, commandString.size() - pos - 2);
//
//         std::vector<std::any> parsedArgs = ParseArguments(argString);
//
//         if (IsValid(command, GetTypeIndices<Args...>())) {
//             return Execute<Ret>(command, std::forward<Args>(args)...);
//         } else {
//             throw std::invalid_argument("Invalid command or argument types");
//         }
//     } else {
//         throw std::invalid_argument("Invalid command string format");
//     }
// }
//
// template <typename... CommandTypes>
// std::vector<std::any> utils::Commands<CommandTypes...>::ParseArguments(const std::string& argString) {
//     std::vector<std::any> args;
//
//     std::istringstream iss(argString);
//     std::string arg;
//     while (iss >> arg) {
//         args.push_back(ParseSingleArgument(arg));
//     }
//
//     return args;
// }
//
// template <typename... CommandTypes>
// std::any utils::Commands<CommandTypes...>::ParseSingleArgument(const std::string& arg) {
//     try {
//         return std::stoi(arg);
//     } catch (const std::invalid_argument& e) {
//         throw std::invalid_argument("Error parsing argument: " + arg);
//     }
// }
//
// template <typename... CommandTypes>
// template <typename T>
// std::type_index utils::Commands<CommandTypes...>::GetTypeIndex() {
//     return std::type_index(typeid(T));
// }

// PathWeaver implementation
utils::PathWeaver::PathWeaver() {}

frc::Trajectory utils::PathWeaver::getTrajectory(std::string_view path) {
  try {
    fs::path path_location = deploy_directory / path;
    return frc::TrajectoryUtil::FromPathweaverJson(path_location.string());
  } catch (const std::exception& e) {
    // Handle the exception appropriately or consider removing the recursive call
    throw;  // Rethrow the exception for now
  }
}

// FollowPath implementation
utils::FollowPath::FollowPath(drivetrain::SwerveDrive* swerve, std::string path, bool flip, std::string name)
    : _swerve(swerve), behaviour::Behaviour(name) {
  _path = pathplanner::PathPlannerPath::fromPathFile(path);

  if (flip) {
    _path = _path->flipPath();
  }

  std::string filePath = frc::filesystem::GetDeployDirectory() + "/pathplanner/paths/" + path + ".path";

  std::cout << filePath << std::endl;

  std::ifstream file(filePath);

  std::string cdata;
  std::string cjson;

  while (file.good()) {
    file >> cdata;
    cjson += cdata;
  }
}