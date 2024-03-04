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
#include "frc/geometry/Transform2d.h"
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

  // cjson.pop_back();

  std::vector<pathplanner::PathPoint> points =
      pathplanner::PathPlannerPath::fromPathFile(path)->getAllPathPoints();

  pathplanner::RotationTarget* lastRot = nullptr;
  units::degree_t rot;

  for (const pathplanner::PathPoint& point : points) {
    if (lastRot != nullptr) {
      rot = point.rotationTarget.value_or(*lastRot).getTarget().Degrees();
    } else {
      rot = point.rotationTarget->getTarget().Degrees();
    }

    pathplanner::RotationTarget t = pathplanner::RotationTarget(0, rot, false);

    lastRot = &t;

    _poses.emplace_back(frc::Pose2d(point.position, rot).TransformBy(frc::Transform2d(-1.37_m, -5.56_m, 0_deg)));
  }
  // _poses = pathplanner::PathPlannerPath::fromPathFile(path)->getPathPoses();
  // wpi::json j = wpi::json::parse(cjson);
  //
  // std::vector<BezierPoint> bezierPoints = createBezierPointsFromJson(j);
  //
  // std::vector<BezierPoint> interpolatedPoints = interpolateBezierPoints(bezierPoints, 0.2);
  //
  // int i = 0;
  // for (const BezierPoint& point : interpolatedPoints) {
  //   frc::Rotation2d a;
  //
  //   if (point.calculateAngle() != NAN) {
  //     a = frc::Rotation2d(units::degree_t{point.calculateAngle()});
  //   } else {
  //     a = _swerve->GetPose().Rotation();
  //   }
  //
  //   frc::Pose2d p = frc::Pose2d(frc::Translation2d(units::meter_t{point.anchor.x},
  //   units::meter_t{point.anchor.y}), a); frc::Pose2d p1 =
  //   frc::Pose2d(frc::Translation2d(units::meter_t{point.nextControl.x},
  //   units::meter_t{point.nextControl.y}), 0_deg); frc::Pose2d p2 =
  //   frc::Pose2d(frc::Translation2d(units::meter_t{point.nextControl.x},
  //   units::meter_t{point.prevControl.y}), 0_deg);
  //
  //   _poses.emplace_back(p);
  //
  //   WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("pathplanner/" + std::to_string(i) +
  //   "/poses/current"), p); WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("pathplanner/" +
  //   std::to_string(i) + "/poses/next"), p1);
  //   WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("pathplanner/" + std::to_string(i) +
  //   "/poses/prev"), p2);
  //
  //   i++;
  // }

  int i = 0;
  for (const frc::Pose2d pose : _poses) {
    WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("pathplanner/poses/" + std::to_string(i)),
                 pose);
    i++;
  }

  CalcTimer();

  _timer.Start();
}

void utils::FollowPath::CalcTimer() {
  frc::Pose2d current_pose = _swerve->GetPose();
  frc::Pose2d target_pose = _poses[_currentPose];

  units::meter_t deltaX = target_pose.Translation().X() - current_pose.Translation().X();
  units::meter_t deltaY = target_pose.Translation().Y() - current_pose.Translation().Y();

  units::meter_t dist = units::meter_t{std::sqrt(std::pow(deltaX.value(), 2) + std::pow(deltaY.value(), 2))};

  _timer.Stop();
  _timer.Reset();
  _time = units::second_t{std::abs(dist.value()) * 2 /*meters per second*/};
  // _time = 20_s;
  _timer.Start();
}

void utils::FollowPath::OnTick(units::second_t dt) {
  _swerve->SetPose(_poses[_currentPose]);
  
  nt::NetworkTableInstance::GetDefault()
      .GetTable("pathplanner")
      ->GetEntry("atPose")
      .SetBoolean(_swerve->IsAtSetPose());
  nt::NetworkTableInstance::GetDefault()
      .GetTable("pathplanner")
      ->GetEntry("timeout")
      .SetDouble(_time.value());
  nt::NetworkTableInstance::GetDefault()
      .GetTable("pathplanner")
      ->GetEntry("currentTime")
      .SetDouble(_timer.Get().value());
  WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("pathplanner/targetPose"),
               _poses[_currentPose]);
  WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("pathplanner/currentPose"),
               _swerve->GetPose());

  std::cout << "Following Path" << std::endl;

  if (_swerve->IsAtSetPose() || _timer.Get() > _time) {
    if (_currentPose + 1 == static_cast<int>(_poses.size())) {
      // _swerve->MakeAtSetPoint();
      _swerve->SetVelocity(frc::ChassisSpeeds());
      _swerve->MakeAtSetPoint();
      SetDone();
    } else {
      _currentPose++;
      CalcTimer();
    }
  }
}

// AutoBuilder implementation
utils::AutoBuilder::AutoBuilder(drivetrain::SwerveDrive* swerve, std::function<bool()> shouldFlipPath,
                                std::string auto_routine, utils::AutoCommands commands)
    : _path(auto_routine), _flip(shouldFlipPath()), _swerve(swerve), _commandsList(std::move(commands)) {
  SetAuto(auto_routine);
}

void utils::AutoBuilder::SetAuto(std::string path) {
  fs::path deploy_directory = frc::filesystem::GetDeployDirectory();

  path = path + ".auto";

  fs::path location = deploy_directory / "pathplanner" / "autos" / path;

  std::cout << location << std::endl;

  std::ifstream file(location);

  std::string cdata;
  std::string cjson;

  while (file.good()) {
    file >> cdata;
    cjson += cdata;
  }

  // cjson.pop_back();

  wpi::json j = wpi::json::parse(cjson);

  _currentPath = &j;
  _startingPose = &j["startingPose"];
  _commands = &j["command"]["data"]["commands"];

  commands = std::vector<std::pair<std::string, std::string>>();

  nt::NetworkTableInstance::GetDefault().GetTable("json")->GetEntry("data").SetString(_currentPath->dump());
  nt::NetworkTableInstance::GetDefault().GetTable("json")->GetEntry("start").SetString(_startingPose->dump());
  nt::NetworkTableInstance::GetDefault().GetTable("json")->GetEntry("commands").SetString(_commands->dump());
  nt::NetworkTableInstance::GetDefault()
      .GetTable("json")
      ->GetEntry("commands_type")
      .SetString(_commands->type_name());

  for (auto c : *_commands) {
    if (c["type"] == "path") {
      commands.push_back(std::make_pair(c["type"], c["data"]["pathName"]));
    }
    if (c["type"] == "named") {
      commands.push_back(std::make_pair(c["type"], c["data"]["name"]));
    }
  }

  nt::NetworkTableInstance::GetDefault().GetTable("commands")->GetEntry("length").SetInteger(commands.size());

  pathplan = behaviour::make<drivetrain::behaviours::DrivebasePoseBehaviour>(
      _swerve, JSONPoseToPose2d(*_startingPose).TransformBy(frc::Transform2d(-1.37_m, -5.56_m, 0_deg)));

  int i = 0;
  int pathamt = 0;
  int commandamt = 0;

  for (std::pair<std::string, std::string> command : commands) {
    nt::NetworkTableInstance::GetDefault()
        .GetTable("commands/" + std::to_string(i))
        ->GetEntry("type")
        .SetString(static_cast<std::string>(command.first));
    nt::NetworkTableInstance::GetDefault()
        .GetTable("commands/" + std::to_string(i))
        ->GetEntry("data")
        .SetString(static_cast<std::string>(command.second));

    if (command.first == "path") {
      auto b = behaviour::make<FollowPath>(_swerve, command.second, _flip);
      std::cout << b->GetName() << std::endl;
      pathplan = pathplan << b;
      pathamt++;
    } else /*(command.first == "command")*/ {
      pathplan = pathplan << _commandsList.Run(command.second);
      commandamt++;
    }

    i++;
  }

  nt::NetworkTableInstance::GetDefault().GetTable("commands")->GetEntry("PathAmt").SetInteger(pathamt);
  nt::NetworkTableInstance::GetDefault().GetTable("commands")->GetEntry("CommandAmt").SetInteger(commandamt);

  WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("startPose"),
               JSONPoseToPose2d(*_startingPose));

  // nt::NetworkTableInstance::GetDefault().GetTable("commands")->GetEntry("behaviours").SetStringArray(pathplan->GetName());
}

void utils::AutoBuilder::Invert() {
  // Implement inversion logic if needed
}

frc::Pose2d utils::AutoBuilder::JSONPoseToPose2d(wpi::json j) {
  // std::cout << j["position"].is_object() << std::endl;
  // std::cout << j << std::endl;

  return frc::Pose2d(units::meter_t{j["position"]["x"]}, units::meter_t{j["position"]["y"]},
                     units::degree_t{j["rotation"]});
  // return frc::Pose2d();
}

std::shared_ptr<behaviour::Behaviour> utils::AutoBuilder::GetAutoPath() {
  std::cout << "Getting Path" << std::endl;

  return pathplan;
}

std::shared_ptr<behaviour::Behaviour> utils::AutoBuilder::GetAutoPath(std::string path) {
  SetAuto(path);
  return GetAutoPath();
}

// template <typename CommandsType>
// behaviour::Behaviour utils::AutoBuilder<CommandsType>::GetPath(std::string path) {
// Implement the logic to return the appropriate behaviour
// You might want to convert it to shared_ptr if needed
// return behaviour::Behaviour(); // Placeholder
// }

// SwerveAutoBuilder implementation
utils::SwerveAutoBuilder::SwerveAutoBuilder(drivetrain::SwerveDrive* swerve, std::string name,
                                            utils::AutoCommands commands)
    : _builder(new AutoBuilder(
          swerve,
          []() {
            // auto alliance = frc::DriverStation::GetAlliance();
            // return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
            return false;
          },
          name, std::move(commands))),
      _swerve(swerve),
      _name(name) {}

void utils::SwerveAutoBuilder::SetAuto(std::string path) {
  _builder->SetAuto(path);
}

std::shared_ptr<behaviour::Behaviour> utils::SwerveAutoBuilder::GetAutoRoutine() {
  return _builder->GetAutoPath();
}

std::shared_ptr<behaviour::Behaviour> utils::SwerveAutoBuilder::GetAutoRoutine(std::string path) {
  return _builder->GetAutoPath(path);
}

drivetrain::SwerveDrive* utils::SwerveAutoBuilder::GetSwerve() {
  return _swerve;
}
