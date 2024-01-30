// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/SwerveDrive.h"

#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/math.h>
#include <units/voltage.h>
<<<<<<< HEAD
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>
=======
>>>>>>> 0029f49 (fix some formatting)

#include <algorithm>
#include <cmath>
#include <iostream>

#include "frc/MathUtil.h"
#include "utils/Util.h"
#include "wpimath/MathShared.h"

using namespace wom;

namespace wom {
namespace drivetrain {

void SwerveModuleConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) const {
  std::array<double, 2> pos{position.X().value(), position.Y().value()};
  table->GetEntry("position").SetDoubleArray(std::span(pos));
  table->GetEntry("wheelRadius").SetDouble(wheelRadius.value());
}

SwerveModule::SwerveModule(std::string path, SwerveModuleConfig config,
                           // SwerveModule::angle_pid_conf_t anglePID,
                           SwerveModule::velocity_pid_conf_t velocityPID)
    :  // _anglePIDController(path + "/pid/angle", anglePID),
<<<<<<< HEAD
      _anglePIDController{frc::PIDController(5, 0, 0, 0.005_s)},
=======
      _anglePIDController{frc::PIDController(4, 0, 0, 0.005_s)},
>>>>>>> 0029f49 (fix some formatting)
      _config(config),
      _velocityPIDController(frc::PIDController(1.2, 0, 0, 0.005_s)),
      _table(nt::NetworkTableInstance::GetDefault().GetTable(path)) {
  // _anglePIDController.SetTolerance(360);
  _anglePIDController.EnableContinuousInput(0, (2 * 3.141592));
  // _anglePIDController.EnableContinuousInput(-3.141592, (3.141592));
  // _anglePIDController.EnableContinuousInput(0, (3.1415 * 2));
  // _anglePIDController.EnableContinuousInput(0, (3.1415));
  // _anglePIDController.SetWrap(0, 3.1415);
}

void SwerveModule::OnStart() {
  // _offset = offset;
  // _config.canEncoder->SetPosition(units::turn_t{0});
  _anglePIDController.Reset();
  // _anglePIDController.EnableContinuousInput(-3.141592, 3.141592);
  // _anglePIDController.EnableContinuousInput(-3.141592, (3.141592));
  _anglePIDController.EnableContinuousInput(0, (2 * 3.141592));

  _velocityPIDController.Reset();
}

void SwerveModule::OnUpdate(units::second_t dt) {
  units::volt_t driveVoltage{0};
  units::volt_t turnVoltage{0};

  switch (_state) {
    case SwerveModuleState::kIdle:
      driveVoltage = 0_V;
      turnVoltage = 0_V;
      break;
    case SwerveModuleState::kPID: {
      auto feedforward = _config.driveMotor.motor.Voltage(
          0_Nm,
          units::radians_per_second_t{_velocityPIDController.GetSetpoint() / _config.wheelRadius.value()});
<<<<<<< HEAD
      // units::radian_t input = units::math::fmod(_config.turnMotor.encoder->GetEncoderPosition(), (2_rad
      // * 3.1415));
      double input = _config.turnMotor.encoder->GetEncoderPosition().value();
=======
      // std::cout << "GetSpeed()" << GetSpeed().value() << std::endl;
      //  units::radian_t input = units::math::fmod(_config.turnMotor.encoder->GetEncoderPosition(), (2_rad
      //  * 3.1415));
      double input = -_config.turnMotor.encoder->GetEncoderPosition().value();
>>>>>>> 0029f49 (fix some formatting)
      _table->GetEntry("/testing/GetEncoderPos").SetDouble(input);
      // _velocityPIDController.SetSetpoint(3);

      driveVoltage = units::volt_t{_velocityPIDController.Calculate(GetSpeed().value())};
      // if (_turnOffset == TurnOffsetValues::forward) {

      // } else if (_turnOffset == TurnOffsetValues::reverse) {
      //   input = input - (3.1415/2);
      //   driveVoltage = -driveVoltage;
      // }
      double demand = _anglePIDController.Calculate(input);
      // if ((_anglePIDController.GetSetpoint() - input) > (3.14159/2)) {
      //   demand *= -1;
      // }
      turnVoltage = units::volt_t{demand};
    } break;
    case wom::drivetrain::SwerveModuleState::kZeroing: {
    } break;
    default:
      std::cerr << "Case not handled" << std::endl;
  }

  units::newton_meter_t torqueLimit = 50_kg / 4 * _config.wheelRadius * _currentAccelerationLimit;
  // units::volt_t voltageMax = _config.driveMotor.motor.Voltage(
  //     torqueLimit, _config.driveMotor.encoder->GetEncoderAngularVelocity());
  // units::volt_t voltageMin = _config.driveMotor.motor.Voltage(
  //     -torqueLimit, _config.driveMotor.encoder->GetEncoderAngularVelocity());

  // driveVoltage =
  //     units::math::max(units::math::min(driveVoltage, voltageMax),
  //     voltageMin);

  driveVoltage = units::math::min(driveVoltage, 7_V);
  turnVoltage = units::math::min(turnVoltage, 4_V);

  // driveVoltage = units::math::min(
  //     units::math::max(driveVoltage, -_driveModuleVoltageLimit),
  //     _driveModuleVoltageLimit);  // was originally 10_V
  units::volt_t turnVoltageMax = 7_V - (driveVoltage * (7_V / 10_V));
  turnVoltage = units::math::min(units::math::max(turnVoltage, -turnVoltageMax), turnVoltageMax);
  // turnVoltage = units::math::min(units::math::max(turnVoltage, -7_V), 7_V);

  _table->GetEntry("TurnVoltage").SetDouble(turnVoltage.value());
  _table->GetEntry("TurnSetpoint").SetDouble(_anglePIDController.GetSetpoint());
  _table->GetEntry("Demand").SetDouble(_config.turnMotor.encoder->GetEncoderPosition().value());
  _table->GetEntry("Error").SetDouble(_anglePIDController.GetPositionError());

  _config.driveMotor.motorController->SetVoltage(driveVoltage);
  _config.turnMotor.motorController->SetVoltage(turnVoltage);

  _table->GetEntry("speed").SetDouble(GetSpeed().value());
<<<<<<< HEAD
  _table->GetEntry("angle").SetDouble(_config.turnMotor.encoder->GetEncoderPosition().value());
=======
  _table->GetEntry("angle").SetDouble(
      _config.turnMotor.encoder->GetEncoderPosition().convert<units::degree>().value());
>>>>>>> 0029f49 (fix some formatting)
  _config.WriteNT(_table->GetSubTable("config"));
}

void SwerveModule::SetTurnOffsetForward() {
  _turnOffset = TurnOffsetValues::forward;
}

void SwerveModule::SetTurnOffsetReverse() {
  _turnOffset = TurnOffsetValues::reverse;
}

void SwerveModule::TurnOffset() {
  _turnOffset = TurnOffsetValues::none;
}

// double SwerveModule::GetCancoderPosition() {
//   return (_config.turnMotor.encoder->GetEncoderPosition().value());
// }

void SwerveModule::SetAccelerationLimit(units::meters_per_second_squared_t limit) {
  _currentAccelerationLimit = limit;
}

void SwerveDrive::SetAccelerationLimit(units::meters_per_second_squared_t limit) {
  for (int motorNumber = 0; motorNumber < 4; motorNumber++) {
    _modules[motorNumber].SetAccelerationLimit(limit);
  }
}

void SwerveModule::SetVoltageLimit(units::volt_t driveModuleVoltageLimit) {
  _driveModuleVoltageLimit = driveModuleVoltageLimit;
}

void SwerveModule::SetIdle() {
  _state = SwerveModuleState::kIdle;
}

void SwerveModule::SetZero(units::second_t dt) {
  SetPID(0_rad, 0_mps, dt);
  _state = SwerveModuleState::kPID;
}

void SwerveModule::SetPID(units::radian_t angle, units::meters_per_second_t speed, units::second_t dt) {
  _state = SwerveModuleState::kPID;
<<<<<<< HEAD
=======
  // @liam start added
  // std::cout << "angle Setpoint: " << _anglePIDController.GetSetpoint() <<
  // std::endl; std::cout << "angle value: " << angle.value() << std::endl;
>>>>>>> 57d11c0 (Shooter pid (#117))

<<<<<<< HEAD
  // double diff = std::abs(_config.turnMotor.encoder->GetEncoderPosition().value() - angle.value());
  // _table->GetEntry("diff").SetDouble(diff);
  // if (std::abs(diff) > (3.14159/2)) {
  //   speed *= -1;
  //   angle = 3.14159_rad - units::radian_t{diff};
  //   _anglePIDController.SetSetpoint(angle.value());
  //   _velocityPIDController.SetSetpoint(speed.value());
  // } else {
  //   _anglePIDController.SetSetpoint(angle.value());
  //   _velocityPIDController.SetSetpoint(speed.value());
  // }
=======
  double diff = std::fmod((_anglePIDController.GetSetpoint() - angle.value()), (2 * 3.1415));
  std::cout << "DIFF value: " << diff << std::endl;
  _table->GetEntry("/Differential value:").SetDouble(diff);
  if (std::abs(diff) >= 3.1415) {
    speed *= -1;
    angle -= 3.1415_rad;
  }
  // @liam end added
>>>>>>> 0029f49 (fix some formatting)

  // if (diff > (3.14159 / 2)) {
  //   speed *= -1;
  //   _anglePIDController.SetSetpoint((3.14159 - (angle.value() -
  //   _config.turnMotor.encoder->GetEncoderPosition().value())));
  //   _velocityPIDController.SetSetpoint(speed.value());
  // } else {

  // double setValue = 3.141592 - (angle.value() - _config.turnMotor.encoder->GetEncoderPosition().value());

  // if (diff > (3.141592/2)) {
  //   if (setValue < 0 ) {
  //     _anglePIDController.SetSetpoint(0);
  //     _velocityPIDController.SetSetpoint(-speed.value());
  //   } else if ( setValue < (2 * 3)) {
  //     _anglePIDController.SetSetpoint(2 * 3.141592  );
  //     _velocityPIDController.SetSetpoint(-speed.value());
  //   } else {
  //     _anglePIDController.SetSetpoint(setValue);
  //     _velocityPIDController.SetSetpoint(-speed.value());
  //   }
  // } else {
  _anglePIDController.SetSetpoint(angle.value());
  _velocityPIDController.SetSetpoint(speed.value());
  // }

  // double currentAngle = _config.turnMotor.encoder->GetEncoderPosition().value();
  // double setpointAngle = closestAngle(currentAngle, _anglePIDController.GetSetpoint());
  // double setpointAngleFlipped = closestAngle(currentAngle, _anglePIDController.GetSetpoint() + 3.1415);
  // _table->GetEntry("/Setpoint angle: ").SetDouble(setpointAngle);
  // _table->GetEntry("/Setpoint angle flipped: ").SetDouble(setpointAngleFlipped);
  // if (std::abs(setpointAngle) <= std::abs(setpointAngleFlipped)) {
  //   // _anglePIDController.SetGain(1.0);
  //   _anglePIDController.SetSetpoint(currentAngle + setpointAngle);
  //   _velocityPIDController.SetSetpoint(speed.value());
  // } else {
  //   // _anglePIDController.SetGain(-1.0);
  //   _velocityPIDController.SetSetpoint(-speed.value());
  //   _anglePIDController.SetSetpoint(currentAngle + setpointAngleFlipped);
  // }
}

// double SwerveModule::closestAngle(double a, double b) {
//   double dir = std::fmod(b, (3.1415 * 2)) - std::fmod(a, (2 * 3.1415));
//   if (std::abs(dir) > 3.1415) {
//     dir = -(((dir < 0) ? -1 : (dir > 0)) * 3.1415) + dir;
//   }
//   return dir;
// }

void SwerveModule::ModuleVectorHandler(frc::ChassisSpeeds speeds) {
  units::meters_per_second_t xVelocityComponent =
      1_mps * (speeds.vx.value() + speeds.omega.value() * _config.position.X().value());
  units::meters_per_second_t yVelocityComponent =
      1_mps * (speeds.vy.value() + speeds.omega.value() * _config.position.Y().value());

  units::meters_per_second_t velocity =
      1_mps * std::sqrt(std::pow(xVelocityComponent.value(), 2) + std::pow(yVelocityComponent.value(), 2));
  units::degree_t angle = 1_rad * std::atan2(yVelocityComponent.value(), xVelocityComponent.value());

  _anglePIDController.SetSetpoint(angle.value());
  _velocityPIDController.SetSetpoint(velocity.value());
}

units::meters_per_second_t SwerveModule::GetSpeed() const {
  units::meters_per_second_t returnVal{_config.driveMotor.encoder->GetVelocityValue() *
                                       _config.wheelRadius.value()};
<<<<<<< HEAD
=======
  // std::cout << returnVal.value() << std::endl;
>>>>>>> 0029f49 (fix some formatting)
  return returnVal;
}

units::meter_t SwerveModule::GetDistance() const {
  return units::meter_t{_config.driveMotor.encoder->GetEncoderPosition().value() *
                        _config.wheelRadius.value()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return frc::SwerveModulePosition{GetDistance(), _config.turnMotor.encoder->GetEncoderPosition()};
}

const SwerveModuleConfig& SwerveModule::GetConfig() const {
  return _config;
}

void SwerveDriveConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) {
  table->GetEntry("mass").SetDouble(mass.value());
}

SwerveDrive::SwerveDrive(SwerveDriveConfig config, frc::Pose2d initialPose)
    : _config(config),
<<<<<<< HEAD
      // _kinematics(_config.modules[1].position, _config.modules[0].position,
      //             _config.modules[2].position, _config.modules[3].position),

      _kinematics(_config.modules[3].position /*1*/, _config.modules[0].position /*0*/,
                  _config.modules[1].position /*2*/, _config.modules[2].position /*3*/),
=======
      _kinematics(_config.modules[0].position, _config.modules[1].position, _config.modules[2].position,
                  _config.modules[3].position),
>>>>>>> 0029f49 (fix some formatting)
      _poseEstimator(
          _kinematics, frc::Rotation2d(0_deg),
          wpi::array<frc::SwerveModulePosition, 4>{frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
                                                   frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
                                                   frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
                                                   frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}}},
          initialPose, _config.stateStdDevs, _config.visionMeasurementStdDevs),
      _anglePIDController{frc::PIDController(1, 0, 0)},
      _xPIDController(config.path + "/pid/x", _config.posePositionPID),
      _yPIDController(config.path + "/pid/y", _config.posePositionPID),
      _table(nt::NetworkTableInstance::GetDefault().GetTable(_config.path)) {
  _anglePIDController.SetTolerance(360);

  int i = 1;
  for (auto cfg : _config.modules) {
    _modules.emplace_back(config.path + "/modules/" + std::to_string(i), cfg,
                          /*config.anglePID,*/ config.velocityPID);
    i++;
  }

  ResetPose(initialPose);
}

frc::ChassisSpeeds FieldRelativeSpeeds::ToChassisSpeeds(const units::radian_t robotHeading) {
  return frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, frc::Rotation2d{robotHeading});
}

void SwerveDrive::OnUpdate(units::second_t dt) {
<<<<<<< HEAD
  _table->GetEntry("/gryo/z").SetDouble(_config.gyro->GetRotation3d().Z().value());
  _table->GetEntry("/gryo/y").SetDouble(_config.gyro->GetRotation3d().Y().value());
  _table->GetEntry("/gryo/x").SetDouble(_config.gyro->GetRotation3d().X().value());
=======
  // std::cout << _config.gyro->GetYaw().GetValue().value() << std::endl;
>>>>>>> 0029f49 (fix some formatting)
  switch (_state) {
    case SwerveDriveState::kZeroing:
      for (auto mod = _modules.begin(); mod < _modules.end(); mod++) {
        // mod->SetZero(dt);
      }
      break;
    case SwerveDriveState::kIdle:
      for (auto mod = _modules.begin(); mod < _modules.end(); mod++) {
        mod->SetIdle();
      }
      break;
    case SwerveDriveState::kPose: {
      _target_fr_speeds.vx = _xPIDController.Calculate(GetPose().X(), dt);
      _target_fr_speeds.vy = _yPIDController.Calculate(GetPose().Y(), dt);
      _target_fr_speeds.omega =
          units::radians_per_second_t{_anglePIDController.Calculate(GetPose().Rotation().Radians().value())};
    }
      [[fallthrough]];
    case SwerveDriveState::kFieldRelativeVelocity:
      _target_speed = _target_fr_speeds.ToChassisSpeeds(GetPose().Rotation().Radians());
      if (isRotateToMatchJoystick) {
        _target_speed.omega = units::radians_per_second_t{
            _anglePIDController.Calculate(GetPose().Rotation().Radians().value())};
      }
      // _target_fr_speeds.vy.value() << std::endl;
      [[fallthrough]];
    case SwerveDriveState::kVelocity: {
      _table->GetEntry("Swerve module VX").SetDouble(_target_speed.vx.value());
      _table->GetEntry("Swerve module VY").SetDouble(_target_speed.vy.value());
      _table->GetEntry("Swerve module Omega").SetDouble(_target_speed.omega.value());
      if (_target_speed.omega.value() > 0) {
        // _modules[0].SetTurnOffsetForward();
        _modules[1].SetTurnOffsetForward();
      } else if (_target_speed.omega.value() < 0) {
        // _modules[0].SetTurnOffsetReverse();
        _modules[1].SetTurnOffsetReverse();
      } else {
        // _modules[0].TurnOffset();
        _modules[1].TurnOffset();
      }

      if (_target_speed.vx > 0_mps || _target_speed.vy > 0_mps) {
        _angle = _target_speed.omega * 1_s;
      }

      auto target_states = _kinematics.ToSwerveModuleStates(_target_speed);
      frc::ChassisSpeeds new_target_speed{_target_speed.vx, _target_speed.vy, -_target_speed.omega};
      auto new_target_states = _kinematics.ToSwerveModuleStates(new_target_speed);
      for (size_t i = 0; i < _modules.size(); i++) {
<<<<<<< HEAD
        if (i == 3) {
          _modules[i].SetPID(new_target_states[i].angle.Radians(), new_target_states[i].speed, dt);
        } else {
          _modules[i].SetPID(target_states[i].angle.Radians(), target_states[i].speed, dt);
          // target_states[i].angle.Radians().value() << std::endl;
        }

        // if (i == 2) {
        //   _modules[i].SetPID(new_target_states[i].angle.Radians(),
        //                     new_target_states[i].speed, dt);
        // }
=======
        _modules[i].SetPID(target_states[i].angle.Radians(), target_states[i].speed, dt);
        // std::cout << "module " << i << ": " <<
        // target_states[i].angle.Radians().value() << std::endl;
>>>>>>> 0029f49 (fix some formatting)
      }
    } break;
    case SwerveDriveState::kIndividualTuning:
      _modules[_mod].SetPID(_angle, _speed, dt);
      break;

    case SwerveDriveState::kTuning:
      for (size_t i = 0; i < _modules.size(); i++) {
        _modules[i].SetPID(_angle, _speed, dt);
      }
      break;
    case SwerveDriveState::kXWheels:
      _modules[0].SetPID(45_deg, 0_mps, dt);
      _modules[1].SetPID(135_deg, 0_mps, dt);
      _modules[2].SetPID(225_deg, 0_mps, dt);
      _modules[3].SetPID(315_deg, 0_mps, dt);
      break;
    case SwerveDriveState::kFRVelocityRotationLock:
      _target_speed.vx = _xPIDController.Calculate(GetPose().X(), dt);
      _target_speed.vy = _yPIDController.Calculate(GetPose().Y(), dt);
      _target_speed.omega =
          units::radians_per_second_t{_anglePIDController.Calculate(GetPose().Rotation().Radians().value())};
      _target_speed = _requestedSpeeds.ToChassisSpeeds(GetPose().Rotation().Radians());
      auto target_states = _kinematics.ToSwerveModuleStates(_target_speed);
      for (size_t i = 0; i < _modules.size(); i++) {
        _modules[i].SetPID(target_states[i].angle.Radians(), target_states[i].speed, dt);
      }
      break;
  }

  for (auto mod = _modules.begin(); mod < _modules.end(); mod++) {
    mod->OnUpdate(dt);
  }

  _poseEstimator.Update(
      _config.gyro->GetRotation2d(),
      wpi::array<frc::SwerveModulePosition, 4>{_modules[0].GetPosition(), _modules[1].GetPosition(),
                                               _modules[2].GetPosition(), _modules[3].GetPosition()});

  utils::WritePose2NT(_table->GetSubTable("estimatedPose"), _poseEstimator.GetEstimatedPosition());
  _config.WriteNT(_table->GetSubTable("config"));
}

void SwerveDrive::SetXWheelState() {
  _state = SwerveDriveState::kXWheels;
}

void SwerveDrive::SetZeroing() {
  _state = SwerveDriveState::kZeroing;
}

void SwerveDrive::SetVoltageLimit(units::volt_t driveVoltageLimit) {
  for (auto mod : _modules) {
    mod.SetVoltageLimit(driveVoltageLimit);
  }
}

// double SwerveDrive::GetModuleCANPosition(int mod) {
//   return _modules[mod].GetCancoderPosition();
// }

void SwerveDrive::OnStart() {
  OnResetMode();

  _modules[0].OnStart();  // front left
  _modules[1].OnStart();  // front right
  _modules[2].OnStart();  // back right
  _modules[3].OnStart();  // back left
}

void SwerveDrive::OnResetMode() {
  _xPIDController.Reset();
  _yPIDController.Reset();
  _anglePIDController.Reset();
}

void SwerveDrive::RotateMatchJoystick(units::radian_t joystickAngle, FieldRelativeSpeeds speeds) {
  // _state = SwerveDriveState::kFRVelocityRotationLock;
  // _anglePIDController.SetSetpoint(joystickAngle);
  // _target_fr_speeds = speeds;
  _state = SwerveDriveState::kFieldRelativeVelocity;
  isRotateToMatchJoystick = true;
  _anglePIDController.SetSetpoint(joystickAngle.value());
  _target_fr_speeds = speeds;
}

void SwerveDrive::SetIdle() {
  _state = SwerveDriveState::kIdle;
}

void SwerveDrive::SetVelocity(frc::ChassisSpeeds speeds) {
  _state = SwerveDriveState::kVelocity;
  _target_speed = speeds;
}

void SwerveDrive::SetIsFieldRelative(bool value) {
  _isFieldRelative = value;
}
bool SwerveDrive::GetIsFieldRelative() {
  return _isFieldRelative;
}

void SwerveDrive::SetIndividualTuning(int mod, units::radian_t angle, units::meters_per_second_t speed) {
  _mod = mod;
  _angle = angle;
  _speed = speed;
  _state = SwerveDriveState::kIndividualTuning;
}

void SwerveDrive::SetTuning(units::radian_t angle, units::meters_per_second_t speed) {
  _angle = angle;
  _speed = speed;
  _state = SwerveDriveState::kTuning;
}

void SwerveDrive::SetFieldRelativeVelocity(FieldRelativeSpeeds speeds) {
  _state = SwerveDriveState::kFieldRelativeVelocity;
  isRotateToMatchJoystick = false;
  _target_fr_speeds = speeds;
}

void SwerveDrive::SetPose(frc::Pose2d pose) {
  _state = SwerveDriveState::kPose;
  _anglePIDController.SetSetpoint(pose.Rotation().Radians().value());
  _xPIDController.SetSetpoint(pose.X());
  _yPIDController.SetSetpoint(pose.Y());
}

bool SwerveDrive::IsAtSetPose() {
  return /*_anglePIDController.IsStable()*/ true && _xPIDController.IsStable() &&
         _yPIDController.IsStable(0.05_m);
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
  _poseEstimator.ResetPosition(
      _config.gyro->GetRotation2d(),
      wpi::array<frc::SwerveModulePosition, 4>{_modules[0].GetPosition(), _modules[1].GetPosition(),
                                               _modules[2].GetPosition(), _modules[3].GetPosition()},
      pose);
}

frc::Pose2d SwerveDrive::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
  _poseEstimator.AddVisionMeasurement(pose, timestamp);
}
}  // namespace drivetrain
}  // namespace wom