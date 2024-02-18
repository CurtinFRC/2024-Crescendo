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
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "drivetrain/behaviours/SwerveBehaviours.h"
#include "frc/Filesystem.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "units/time.h"

using namespace wom;

utils::Commands::Commands(std::initializer_list<std::pair<std::string, std::function<void(std::vector<std::any>)>>> init) : _commands{} {
  for (const auto& pair : init) {
    _commands.emplace_back(pair.first, std::vector<std::type_index>{typeid(std::vector<std::any>)}, pair.second);
  }
}

void utils::Commands::Execute(std::string command, std::vector<std::any> args) {
  auto it = std::find_if(_commands.begin(), _commands.end(),
                         [command](const auto& cmd) { return cmd.name == command; });

  if (it != _commands.end()) {
    if (IsValid(command, args)) {
      it->function(args);  
    } else {
      std::cerr << "Invalid arguments for command: " << command << std::endl;
    }
  } else {
    std::cerr << "Unknown command: " << command << std::endl;
  }
}

void utils::Commands::Run(std::string command) {
  command.erase(std::remove_if(command.begin(), command.end(), ::isspace), command.end());

  size_t openParenPos = command.find('(');
  size_t closeParenPos = command.find(')');

  if (openParenPos != std::string::npos && closeParenPos != std::string::npos &&
      closeParenPos > openParenPos) {
    std::string commandName = command.substr(0, openParenPos);
    std::string argString = command.substr(openParenPos + 1, closeParenPos - openParenPos - 1);

    std::vector<std::any> args = ParseArguments(argString);

    Execute(commandName, args);
  } else {
    std::cerr << "Invalid command format: " << command << std::endl;
  }
}

std::vector<std::any> utils::Commands::ParseArguments(const std::string& argString) {
  std::vector<std::any> args;

  std::istringstream iss(argString);
  std::string token;

  while (std::getline(iss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
    args.push_back(std::any(token));
  }

  return args;
}

bool utils::Commands::IsValid(std::string command, const std::vector<std::any> args) {
  auto it = std::find_if(_commands.begin(), _commands.end(),
                         [command](const auto& cmd) { return cmd.name == command; });

  if (it != _commands.end()) {
    const auto& expectedTypes = it->argumentTypes;

    if (args.size() != expectedTypes.size()) {
      return false;
    }

    for (size_t i = 0; i < args.size(); ++i) {
      if (args[i].type() != expectedTypes[i]) {
        return false;
      }
    }

    return true;
  } else {
    return false;
  }
}

utils::Pathplanner::Pathplanner() {}

frc::Trajectory utils::Pathplanner::getTrajectory(std::string_view path) {
  try {
    fs::path path_location = deploy_directory / path;
    return frc::TrajectoryUtil::FromPathweaverJson(path_location.string());
  } catch (std::exception& e) {
    return getTrajectory(path);
  }
}

utils::FollowPath::FollowPath(drivetrain::SwerveDrive* swerve, std::string path, bool flip)
    : _swerve(swerve) {
  _path = pathplanner::PathPlannerPath::fromPathFile(path);

  if (flip) {
    _path = _path->flipPath();
  }

  _poses = _path->getPathPoses();
}

void utils::FollowPath::OnTick(units::second_t dt) {
  _swerve->SetPose(_poses[_currentPose]);

  if (_swerve->IsAtSetPose()) {
    if (_currentPose == static_cast<int>(_poses.size())) {
      SetDone();
    } else {
      _currentPose++;
    }
  }
};

utils::RunCommand::RunCommand(std::string command, Commands commands) : _command(command), _commands(commands) {}

void utils::RunCommand::OnTick(units::second_t dt) {
  _commands.Run(_command);

  SetDone();
}

utils::AutoBuilder::AutoBuilder(drivetrain::SwerveDrive* swerve, std::function<bool()> shouldFlipPath,
                                std::string auto_routine, Commands commands)
    : _swerve(swerve), _flip(shouldFlipPath()), _path(auto_routine), _commandsList(commands) {
  SetAuto(auto_routine);
};

void utils::AutoBuilder::SetAuto(std::string_view path) {
  fs::path deploy_directory = frc::filesystem::GetDeployDirectory();
  fs::path location = deploy_directory / path;

  std::ifstream file(location);

  wpi::json j;

  file >> j;

  _currentPath = &j;
  _startingPose = &j["startingPose"];
  _commands = &j["command"]["data"]["commands"];
}

void utils::AutoBuilder::Invert() {}

frc::Pose2d utils::AutoBuilder::JSONPoseToPose2d(wpi::json j) {
  return frc::Pose2d(units::meter_t{j["position"]["x"]}, units::meter_t{j["position"]["y"]},
                     units::degree_t{j["rotation"]});
}

std::shared_ptr<behaviour::Behaviour> utils::AutoBuilder::GetAutoPath() {
  behaviour::Behaviour::ptr path = behaviour::make<drivetrain::behaviours::DrivebasePoseBehaviour>(
      _swerve, JSONPoseToPose2d(*_startingPose));

  for (auto& command : *_commands) {
    if (command["type"] == "path") {
      path << behaviour::make<FollowPath>(_swerve, command["data"]["pathName"], _flip);
    } else if (command["type"] == "command") {
      path << behaviour::make<RunCommand>(command["data"]["name"], _commandsList);
    }
  }

  return path;
}

utils::SwerveAutoBuilder::SwerveAutoBuilder(wom::drivetrain::SwerveDrive* swerve, std::string name)
    : _swerve(swerve), _name(name) {
  _builder = new AutoBuilder(
      swerve,
      []() {
        auto alliance = frc::DriverStation::GetAlliance();

        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }

        return false;
      },
      "taxi",
      Commands());
}
