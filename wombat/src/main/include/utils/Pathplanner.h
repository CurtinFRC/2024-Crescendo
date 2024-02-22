// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include <algorithm>
#include <any>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <vector>
#include <stdexcept>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "pathplanner/lib/path/PathPlannerPath.h"
#include "wpi/json_fwd.h"

namespace wom {
namespace utils {

// template <typename Ret, typename... Args>
// class ConcreteCommand;

// template <typename Ret, typename... Args>
// class CommandBase {
// public:
  // virtual ~CommandBase() = default;
    // virtual std::shared_ptr<behaviour::Behaviour> Execute(Args...) const = 0;
    // virtual bool IsValid(std::vector<std::type_index> argTypes) const = 0;
    // virtual std::string GetName() const = 0;
// };

// template <typename Ret, typename... Args>
// class Command : public CommandBase<Ret, Args...>{
// public:
    // using FunctionType = std::function<Ret(Args...)>;
//
    // Command(std::string name, FunctionType func);

  // std::shared_ptr<behaviour::Behaviour> Execute(Args... args) const;
    // Ret Execute(Args... args) const;

    // std::string GetName() const override;

    // bool IsValid(std::vector<std::type_index> argTypes) const;

// private:
    // std::string name_;
    // FunctionType function_;
    // std::vector<std::type_index> argumentTypes_;
// };
// template <typename BehaviorType>
//     class Command {
//     public:
//         using BehaviorPtr = std::shared_ptr<BehaviorType>;
//
//         Command(const std::string& name, std::function<BehaviorPtr()> func)
//             : name_(name), func_(func) {}
//
//         BehaviorPtr Execute() const {
//             return func_();
//         }
//
//         const std::string& GetName() const {
//             return name_;
//         }
//
//         bool IsValid() const {
//             return func_ != nullptr;
//         }
//
//     private:
//         std::string name_;
//         std::function<BehaviorPtr()> func_;
//     };
//
// class Commands {
// public:
//     template <typename... CommandArgs>
//     Commands(CommandArgs&&... commands);
//
//     template <typename Ret, typename... Args>
//     Ret Execute(std::string command, Args&&... args);
//
//     bool IsValid(std::string command, std::vector<std::type_index> argTypes) const;
//
//     template <typename Ret, typename... Args>
//     Ret Run(std::string commandString, Args&&... args);
//
// private:
//     std::tuple<CommandTypes...> commands_;
//
//     std::vector<std::any> ParseArguments(const std::string& argString);
//     std::any ParseSingleArgument(const std::string& arg);
//
//     template <typename T>
//     static std::type_index GetTypeIndex();
//     
//   template <typename... Args>
//   std::vector<std::type_index> GetTypeIndices() {
//     return {GetTypeIndex<Args>()...};
//   }
// };
// template <typename BehaviorType>
//     class Command {
//     public:
//         using BehaviorPtr = std::shared_ptr<behaviour::Behaviour>;
//
//         template <typename Function>
//         Command(Function&& func, std::string name)
//             : func_{std::make_shared<Function>(std::forward<Function>(func))}, name_{std::move(name)} {}
//
//         BehaviorPtr Execute() const  {
//             return func_();
//         }
//
//         std::string GetName() const  {
//             return name_;
//         }
//
//     private:
//         std::function<std::shared_ptr<behaviour::Behaviour()>> func_;
//         std::string name_;
//     };

    class Commands {
    public:

        // template <typename... CommandTypes>
        // Commands(CommandTypes&&... commands)
            // : commands_{std::make_shared<Command<BehaviorType, CommandTypes>>(std::forward<CommandTypes>(commands))...} {}

        Commands(std::vector<std::pair<std::string, std::function<std::shared_ptr<behaviour::Behaviour>()>>> commands) : commands_(std::move(commands)) {};

  std::shared_ptr<behaviour::Behaviour> Execute(std::string command) {
    auto it = std::find_if(commands_.begin(), commands_.end(), [&command](const auto& cmd) {
        return cmd.first == command;
    });

    if (it != commands_.end()) {
        return (it->second)();
    } else {
        throw std::invalid_argument("Command not found");
    }
  }

        // bool IsValid(std::string command) const {
            // auto it = std::find_if(commands_.begin(), commands_.end(), [&command](const auto& cmd) {
                // return cmd->f == command;
            // });

            // return it != commands_.end();
        // }

  std::shared_ptr<behaviour::Behaviour> Run(std::string commandString) {
          return Execute(commandString);
  }

    private:
        std::vector<std::pair<std::string, std::function<std::shared_ptr<behaviour::Behaviour>()>>> commands_ = {};
    };

// template <typename Ret, typename... Args>
// class ConcreteCommand : public CommandBase<Ret, Args...> {
// public:
  // using CommandType = Command<Ret, Args...>;

    // ConcreteCommand(std::string name, typename CommandType::FunctionType func)
        // : command_(std::move(name), std::move(func)) {}
//
//     void Execute(std::vector<std::any> args) const override {
//         // Convert args to the expected types and call Execute
//         if (args.size() == sizeof...(Args)) {
//             command_.Execute(std::any_cast<Args>(args[0])...);
//         } else {
//             throw std::invalid_argument("Incorrect number of arguments for command");
//         }
//     }
//
//     bool IsValid(std::vector<std::type_index> argTypes) const override {
//         return command_.IsValid(argTypes);
//     }
//
//     std::string GetName() const override {
//         return command_.GetName();
//     }
//
// private:
//     Command<Ret, Args...> command_;
// };

class PathWeaver {
 public:
  PathWeaver();

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

class AutoBuilder {
 public:
  AutoBuilder(drivetrain::SwerveDrive* swerve, std::function<bool()> shouldFlipPath, std::string autoRoutine, Commands commands);

  void Invert();
  void SetAuto(std::string_view path);

  std::shared_ptr<behaviour::Behaviour> GetAutoPath();
  std::shared_ptr<behaviour::Behaviour> GetAutoPath(std::string path);

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
  SwerveAutoBuilder(drivetrain::SwerveDrive* swerve, std::string name, Commands commands);
  
  void SetAuto(std::string path);
  std::shared_ptr<behaviour::Behaviour> GetAutoRoutine();
  std::shared_ptr<behaviour::Behaviour> GetAutoRoutine(std::string path);  

 private:
  AutoBuilder* _builder;
  drivetrain::SwerveDrive* _swerve;
  std::string _name;
};

}  // namespace utils
}  // namespace wom
