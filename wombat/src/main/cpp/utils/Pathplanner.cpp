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
#include <initializer_list>
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
#include "wpi/json_fwd.h"

using namespace wom;

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
//     : commands_{std::make_shared<utils::ConcreteCommand<CommandTypes>>(std::forward<CommandArgs>(commands))...} {}
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
// bool utils::Commands<CommandTypes...>::IsValid(std::string command, std::vector<std::type_index> argTypes) const {
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
        throw; // Rethrow the exception for now
    }
}

// FollowPath implementation
utils::FollowPath::FollowPath(drivetrain::SwerveDrive* swerve, std::string path, bool flip)
    : _swerve(swerve) {
    _path = pathplanner::PathPlannerPath::fromPathFile(path);

    if (flip) {
        _path = _path->flipPath();
    }

    _poses = _path->getPathPoses();
}

void utils::FollowPath::OnTick(units::time::second_t dt) {
    _swerve->SetPose(_poses[_currentPose]);

    if (_swerve->IsAtSetPose()) {
        if (_currentPose == static_cast<int>(_poses.size())) {
            SetDone();
        } else {
            _currentPose++;
        }
    }
}

// AutoBuilder implementation
utils::AutoBuilder::AutoBuilder(drivetrain::SwerveDrive* swerve,
                                              std::function<bool()> shouldFlipPath,
                                              std::string auto_routine,
                                              utils::Commands commands)
    : _path(auto_routine),
      _flip(shouldFlipPath()),
      _swerve(swerve),
      _commandsList(std::move(commands)) {
    SetAuto(auto_routine);
}

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

void utils::AutoBuilder::Invert() {
    // Implement inversion logic if needed
}

frc::Pose2d utils::AutoBuilder::JSONPoseToPose2d(wpi::json j) {
    return frc::Pose2d(units::meter_t{j["position"]["x"]},
                       units::meter_t{j["position"]["y"]},
                       units::degree_t{j["rotation"]});
}

std::shared_ptr<behaviour::Behaviour> utils::AutoBuilder::GetAutoPath() {
    behaviour::Behaviour::ptr path = behaviour::make<drivetrain::behaviours::DrivebasePoseBehaviour>(
        _swerve, JSONPoseToPose2d(*_startingPose));

    for (auto& command : *_commands) {
        if (command["type"] == "path") {
            path << behaviour::make<FollowPath>(_swerve, command["data"]["pathName"], _flip);
        } else if (command["type"] == "command") {
            path << _commandsList.Run(command["data"]["name"]);
        }
    }

    return path;
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
utils::SwerveAutoBuilder::SwerveAutoBuilder(drivetrain::SwerveDrive* swerve,
                                                          std::string name,
                                                          utils::Commands commands)
    : _builder(new AutoBuilder(
          swerve,
          []() {
              auto alliance = frc::DriverStation::GetAlliance();
              return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
          },
          name,
          std::move(commands))),
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
