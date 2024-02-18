// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include <any>
#include <string>
#include <typeindex>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "drivetrain/behaviours/SwerveBehaviours.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "pathplanner/lib/util/PIDConstants.h"
#include "wpi/json_fwd.h"

namespace wom {
namespace utils {

class Commands {
 public:
  
  Commands(std::initializer_list<std::pair<std::string, std::function<void(std::vector<std::any>)>>> init);

  Commands() = default;
  
  void Execute(std::string command, std::vector<std::any> args);

  void Run(std::string command);

  bool IsValid(std::string command, std::vector<std::any> args);

 private:
  struct CommandData {
    std::string name;
    std::vector<std::type_index> argumentTypes;
    std::function<void(std::vector<std::any>)> function;
  };

  std::vector<CommandData> _commands;  // Changed name to _commands

  // Helper function to parse command arguments from a string
  std::vector<std::any> ParseArguments(const std::string& argString);
};

class Pathplanner {
 public:
  Pathplanner();

  frc::Trajectory getTrajectory(std::string_view path);

 private:
  fs::path deploy_directory = frc::filesystem::GetDeployDirectory();
};

class FollowPath : public behaviour::Behaviour {
 public:
  FollowPath(drivetrain::SwerveDrive* swerve, std::string path, bool flip = false);

  void OnTick(units::time::second_t dt) override;

 private:
  std::string _pathName;
  std::shared_ptr<pathplanner::PathPlannerPath> _path;
  std::vector<frc::Pose2d> _poses;

  drivetrain::SwerveDrive* _swerve;

  int _currentPose = 0;
};

class RunCommand : public behaviour::Behaviour {
public:
  RunCommand(std::string command, Commands commands);

  void OnTick(units::time::second_t dt) override;

  private:
  std::string _command;
  Commands _commands;
};

class AutoBuilder {
 public:
  AutoBuilder(drivetrain::SwerveDrive* swerve, std::function<bool()> shouldFlipPath, std::string autoRoutine, Commands commands);

  void Invert();
  void SetAuto(std::string_view path);

  std::shared_ptr<behaviour::Behaviour> GetAutoPath();
  behaviour::Behaviour GetPath(std::string path);

  frc::Pose2d JSONPoseToPose2d(wpi::json j);

 private:
  std::string _path;
  bool _flip;
  drivetrain::SwerveDrive* _swerve;

  Commands _commandsList;

  wpi::json* _currentPath;
  wpi::json* _startingPose;
  wpi::json* _commands;
};

class SwerveAutoBuilder {
 public:
  SwerveAutoBuilder(wom::drivetrain::SwerveDrive* swerve, std::string name);

 private:
  AutoBuilder* _builder;
  wom::drivetrain::SwerveDrive* _swerve;
  std::string _name;
};
}  // namespace utils
}  // namespace wom
