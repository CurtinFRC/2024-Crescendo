// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/SwerveDrive.h"

#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/math.h>
#include <units/voltage.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "frc/MathUtil.h"
#include "units/velocity.h"
#include "utils/Util.h"

using namespace wom;

namespace wom {
namespace drivetrain {

PIDController::PIDController(double Kp, double Ki, double Kd, units::second_t period)
    : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_period(period) {
  bool invalidGains = false;
  if (Kp < 0.0) {
    wpi::math::MathSharedStore::ReportError("Kp must be a non-negative number, got {}!", Kp);
    invalidGains = true;
  }
  if (Ki < 0.0) {
    wpi::math::MathSharedStore::ReportError("Ki must be a non-negative number, got {}!", Ki);
    invalidGains = true;
  }
  if (Kd < 0.0) {
    wpi::math::MathSharedStore::ReportError("Kd must be a non-negative number, got {}!", Kd);
    invalidGains = true;
  }
  if (invalidGains) {
    m_Kp = 0.0;
    m_Ki = 0.0;
    m_Kd = 0.0;
    wpi::math::MathSharedStore::ReportWarning("PID gains defaulted to 0.");
  }

  if (period <= 0_s) {
    wpi::math::MathSharedStore::ReportError("Controller period must be a positive number, got {}!",
                                            period.value());
    m_period = 20_ms;
    wpi::math::MathSharedStore::ReportWarning("Controller period defaulted to 20ms.");
  }
  static int instances = 0;
  instances++;

  wpi::math::MathSharedStore::ReportUsage(wpi::math::MathUsageId::kController_PIDController2, instances);
  wpi::SendableRegistry::Add(this, "PIDController", instances);
}

void PIDController::SetPID(double Kp, double Ki, double Kd) {
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;
}

void PIDController::SetP(double Kp) {
  m_Kp = Kp;
}

void PIDController::SetI(double Ki) {
  m_Ki = Ki;
}

void PIDController::SetD(double Kd) {
  m_Kd = Kd;
}

void PIDController::SetIZone(double iZone) {
  if (iZone < 0) {
    wpi::math::MathSharedStore::ReportError("IZone must be a non-negative number, got {}!", iZone);
  }
  m_iZone = iZone;
}

double PIDController::GetP() const {
  return m_Kp;
}

double PIDController::GetI() const {
  return m_Ki;
}

double PIDController::GetD() const {
  return m_Kd;
}

double PIDController::GetIZone() const {
  return m_iZone;
}

units::second_t PIDController::GetPeriod() const {
  return m_period;
}

double PIDController::GetPositionTolerance() const {
  return m_positionTolerance;
}

double PIDController::GetVelocityTolerance() const {
  return m_velocityTolerance;
}

void PIDController::SetSetpoint(double setpoint) {
  m_setpoint = setpoint;
  m_haveSetpoint = true;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError = frc::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period.value();
}

double PIDController::GetSetpoint() const {
  return m_setpoint;
}

bool PIDController::AtSetpoint() const {
  return m_haveMeasurement && m_haveSetpoint && std::abs(m_positionError) < m_positionTolerance &&
         std::abs(m_velocityError) < m_velocityTolerance;
}

void PIDController::EnableContinuousInput(double minimumInput, double maximumInput) {
  m_continuous = true;
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
}

void PIDController::DisableContinuousInput() {
  m_continuous = false;
}

bool PIDController::IsContinuousInputEnabled() const {
  return m_continuous;
}

void PIDController::SetIntegratorRange(double minimumIntegral, double maximumIntegral) {
  m_minimumIntegral = minimumIntegral;
  m_maximumIntegral = maximumIntegral;
}

void PIDController::SetTolerance(double positionTolerance, double velocityTolerance) {
  m_positionTolerance = positionTolerance;
  m_velocityTolerance = velocityTolerance;
}

double PIDController::GetPositionError() const {
  return m_positionError;
}

double PIDController::GetVelocityError() const {
  return m_velocityError;
}

double PIDController::Calculate(double measurement) {
  m_measurement = measurement;
  m_prevError = m_positionError;
  m_haveMeasurement = true;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError = frc::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period.value();

  // If the absolute value of the position error is outside of IZone, reset the
  // total error
  if (std::abs(m_positionError) > m_iZone) {
    m_totalError = 0;
  } else if (m_Ki != 0) {
    m_totalError = std::clamp(m_totalError + m_positionError * m_period.value(), m_minimumIntegral / m_Ki,
                              m_maximumIntegral / m_Ki);
  }

  // double absError = m_setpoint - m_measurement;
  // if (absError < 3.1415) {
  //   m_positionError = absError;
  // } else {
  //   m_positionError = absError - (3.1415 * 2);
  // }

  return m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
}

double PIDController::Calculate(double measurement, double setpoint) {
  m_setpoint = setpoint;
  m_haveSetpoint = true;
  return Calculate(measurement);
}

void PIDController::Reset() {
  m_positionError = 0;
  m_prevError = 0;
  m_totalError = 0;
  m_velocityError = 0;
  m_haveMeasurement = false;
}

void PIDController::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("PIDController");
  builder.AddDoubleProperty(
      "p", [this] { return GetP(); }, [this](double value) { SetP(value); });
  builder.AddDoubleProperty(
      "i", [this] { return GetI(); }, [this](double value) { SetI(value); });
  builder.AddDoubleProperty(
      "d", [this] { return GetD(); }, [this](double value) { SetD(value); });
  builder.AddDoubleProperty(
      "izone", [this] { return GetIZone(); }, [this](double value) { SetIZone(value); });
  builder.AddDoubleProperty(
      "setpoint", [this] { return GetSetpoint(); }, [this](double value) { SetSetpoint(value); });
}

void SwerveModuleConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) const {
  #ifdef DEBUG
  std::array<double, 2> pos{position.X().value(), position.Y().value()};
  table->GetEntry("position").SetDoubleArray(std::span(pos));
  table->GetEntry("wheelRadius").SetDouble(wheelRadius.value());
  #endif
}

SwerveModule::SwerveModule(std::string path, SwerveModuleConfig config,
                           // SwerveModule::angle_pid_conf_t anglePID,
                           SwerveModule::velocity_pid_conf_t velocityPID)
    :  //_anglePIDController(path + "/pid/angle", anglePID),
      _anglePIDController{frc::PIDController(5, 0, 0, 0.005_s)},
      _config(config),
      _velocityPIDController(frc::PIDController(1.2, 0, 0, 0.005_s)),
      _table(nt::NetworkTableInstance::GetDefault().GetTable(path)) {
  // _anglePIDController.SetTolerance(360);
  // _anglePIDController.EnableContinuousInput(-3.1415, (2 * 3.1415));
  _anglePIDController.EnableContinuousInput(0, (2 * 3.1415));
}

void SwerveModule::OnStart() {
  // _offset = offset;
  // _config.canEncoder->SetPosition(units::turn_t{0});
  _anglePIDController.Reset();
  _anglePIDController.EnableContinuousInput(0, 2 * 3.14159);
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
      // units::radian_t input = units::math::fmod(_config.turnMotor.encoder->GetEncoderPosition(), (2_rad
      // * 3.1415));
      double input = _config.turnMotor.encoder->GetEncoderPosition().value();
    #ifdef DEBUG
      _table->GetEntry("/testing/GetEncoderPos").SetDouble(input);
    #endif
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
  //     units::math::max(units::math::min(driveVoltage, voltageMax), voltageMin);

  driveVoltage = units::math::min(driveVoltage, 7_V);
  turnVoltage = units::math::min(turnVoltage, 6_V);

  // driveVoltage = units::math::min(
  //     units::math::max(driveVoltage, -_driveModuleVoltageLimit),
  //     _driveModuleVoltageLimit);  // was originally 10_V
  units::volt_t turnVoltageMax = 7_V - (driveVoltage * (7_V / 10_V));
  turnVoltage = units::math::min(units::math::max(turnVoltage, -turnVoltageMax), turnVoltageMax);
  // turnVoltage = units::math::min(units::math::max(turnVoltage, -7_V), 7_V);

  #ifdef DEBUG
  _table->GetEntry("TurnVoltage").SetDouble(turnVoltage.value());
  _table->GetEntry("TurnSetpoint").SetDouble(_anglePIDController.GetSetpoint());
  _table->GetEntry("Demand").SetDouble(_config.turnMotor.encoder->GetEncoderPosition().value());
  _table->GetEntry("Error").SetDouble(_anglePIDController.GetPositionError());
  #endif

  _config.driveMotor.motorController->SetVoltage(driveVoltage);
  _config.turnMotor.motorController->SetVoltage(turnVoltage);

   #ifdef DEBUG
  _table->GetEntry("speed").SetDouble(GetSpeed().value());
  _table->GetEntry("angle").SetDouble(_config.turnMotor.encoder->GetEncoderPosition().value());
  _config.WriteNT(_table->GetSubTable("config"));
  #endif
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

  // double diff = std::abs(_config.turnMotor.encoder->GetEncoderPosition().value() - angle.value());
  // _table->GetEntry("diff").SetDouble(diff);
  // if (std::abs(diff) > (3.14159/2)) {
  //   speed *= -1;
  //   angle = 3.14159_rad - units::radian_t{diff};
  //   _anglePIDController.SetSetpoint(angle.value());
  //   _velocityPIDController.SetSetpoint(speed.value());
  // } else {
  _anglePIDController.SetSetpoint(angle.value());
  _velocityPIDController.SetSetpoint(speed.value());
  // }
}

void SwerveModule::ModuleVectorHandler(frc::ChassisSpeeds speeds) {
  units::meters_per_second_t xVelocityComponent =
      1_mps * (speeds.vx.value() + speeds.omega.value() * _config.position.X().value());
  units::meters_per_second_t yVelocityComponent =
      1_mps * (speeds.vy.value() + speeds.omega.value() * _config.position.Y().value());

  units::meters_per_second_t velocity =
      1_mps * std::sqrt(std::pow(xVelocityComponent.value(), 2) + std::pow(yVelocityComponent.value(), 2));
  units::radian_t angle = 1_rad * std::atan2(yVelocityComponent.value(), xVelocityComponent.value());

  _anglePIDController.SetSetpoint(angle.value());
  _velocityPIDController.SetSetpoint(velocity.value());
}

units::meters_per_second_t SwerveModule::GetSpeed() const {
  units::meters_per_second_t returnVal{_config.driveMotor.encoder->GetVelocityValue() *
                                       _config.wheelRadius.value()};
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
  #ifdef DEBUG
  table->GetEntry("mass").SetDouble(mass.value());
  #endif
}

SwerveDrive::SwerveDrive(SwerveDriveConfig config, frc::Pose2d initialPose)
    : _config(config),
      _kinematics(_config.modules[3].position, _config.modules[0].position, _config.modules[1].position,
                  _config.modules[2].position),
      _poseEstimator(
          _kinematics, frc::Rotation2d(0_deg),
          wpi::array<frc::SwerveModulePosition, 4>{frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
                                                   frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
                                                   frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}},
                                                   frc::SwerveModulePosition{0_m, frc::Rotation2d{0_deg}}},
          initialPose, _config.stateStdDevs, _config.visionMeasurementStdDevs),
      _anglePIDController{PIDController(0, 0, 0)},
      _xPIDController(PIDController(4, 0.1, 0)),
      _yPIDController(PIDController(4, 0.1, 0)),
      _turnPIDController(PIDController(7, 0, 0)),
      // _xPIDController(std::string path, config_t initialGains)
      // _xPIDController(config.path + "/pid/x", _config.posePositionPID),
      // _yPIDController(config.path + "/pid/y", _config.posePositionPID),
      _table(nt::NetworkTableInstance::GetDefault().GetTable(_config.path)) {
  _anglePIDController.SetTolerance(360);
  _anglePIDController.EnableContinuousInput(0, 2 * 3.14159);
  // _anglePIDController.EnableContinuousInput(-3.14159, 3.14159);

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
  #ifdef DEBUG
  _table->GetEntry("/gryo/z").SetDouble(_config.gyro->GetRotation3d().Z().value());
  _table->GetEntry("/gryo/y").SetDouble(_config.gyro->GetRotation3d().Y().value());
  _table->GetEntry("/gryo/x").SetDouble(_config.gyro->GetRotation3d().X().value());
  #endif
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

    case SwerveDriveState::kAngle: {
      _turnPIDController.SetSetpoint(_reqAngle.value());

      double direction = _turnPIDController.Calculate(_config.gyro->GetRotation2d().Radians().value());

      for (size_t i = 0; i < _modules.size(); i++) {
        switch (i) {
          case 0: {
            _modules[i].SetPID(135_deg, units::meters_per_second_t{-direction}, dt);
          } break;

          case 1: {
            _modules[i].SetPID(45_deg, units::meters_per_second_t{-direction}, dt);
          } break;

          case 2: {
            _modules[i].SetPID(135_deg, units::meters_per_second_t{direction}, dt);
          } break;

          case 3: {
            _modules[i].SetPID(45_deg, units::meters_per_second_t{direction}, dt);
          } break;
        }
      }

  #ifdef DEBUG
      _table->GetEntry("direction").SetDouble(direction);
      _table->GetEntry("_reqAngle").SetDouble(_reqAngle.value());
#endif
    } break;
    case SwerveDriveState::kPose: {
      // _target_fr_speeds.vx = _xPIDController.Calculate(GetPose().X(), dt);
      // _target_fr_speeds.vy = _yPIDController.Calculate(GetPose().Y(), dt);
      _target_fr_speeds.vx = units::meters_per_second_t{_xPIDController.Calculate(GetPose().X().value())};
      _target_fr_speeds.vy = units::meters_per_second_t{_yPIDController.Calculate(GetPose().Y().value())};
      _target_fr_speeds.omega = units::radians_per_second_t{_anglePIDController.Calculate(GetPose().Rotation().Radians().value())};

  #ifdef DEBUG
      _table->GetEntry("Swerve vx").SetDouble(_target_fr_speeds.vx.value());
      _table->GetEntry("Swerve vy").SetDouble(_target_fr_speeds.vy.value());
      _table->GetEntry("Swerve omega").SetDouble(_target_fr_speeds.omega.value());
#endif
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
  #ifdef DEBUG
      _table->GetEntry("Swerve module VX").SetDouble(_target_speed.vx.value());
      _table->GetEntry("Swerve module VY").SetDouble(_target_speed.vy.value());
      _table->GetEntry("Swerve module Omega").SetDouble(_target_speed.omega.value());
#endif
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

      frc::ChassisSpeeds new_target_speed{_target_speed.vx, _target_speed.vy, -_target_speed.omega};
      auto target_states = _kinematics.ToSwerveModuleStates(_target_speed);
      auto new_target_states = _kinematics.ToSwerveModuleStates(new_target_speed);
      for (size_t i = 0; i < _modules.size(); i++) {
        if (i == 0 || i == 2 || i == 1) {
          _modules[i].SetPID(new_target_states[i].angle.Radians(), new_target_states[i].speed, dt);
        } else {
          _modules[i].SetPID(target_states[i].angle.Radians(), target_states[i].speed, dt);
        }
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
      // _target_speed.vx = _xPIDController.Calculate(GetPose().X(), dt);
      // _target_speed.vy = _yPIDController.Calculate(GetPose().Y(), dt);
      _target_fr_speeds.vx = units::meters_per_second_t{_xPIDController.Calculate(GetPose().X().value())};
      _target_fr_speeds.vx = units::meters_per_second_t{_xPIDController.Calculate(GetPose().Y().value())};

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
      _config.gyro->GetRotation2d() /* - 1_rad*/,
      wpi::array<frc::SwerveModulePosition, 4>{_modules[3].GetPosition(), _modules[0].GetPosition(),
                                               _modules[1].GetPosition(), _modules[2].GetPosition()});

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

  _anglePIDController.EnableContinuousInput(0, 2 * 3.14159);
  // _anglePIDController.EnableContinuousInput(-3.14159, 3.14159);

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
  if (pose.X() > 4_m) {
      pose = frc::Pose2d(0_m, pose.Y(), pose.Rotation());
    }
    if (pose.Y() > 4_m) {
      pose = frc::Pose2d(pose.X(), 0_m, pose.Rotation());
    }

    if (pose.X() < -4_m) {
      pose = frc::Pose2d(0_m, pose.Y(), pose.Rotation());
    }
    if (pose.Y() < -4_m) {
      pose = frc::Pose2d(pose.X(), 0_m, pose.Rotation());
  }
  _anglePIDController.SetSetpoint(pose.Rotation().Radians().value()/* - 3.14159*/);
  _xPIDController.SetSetpoint(pose.X().value());
  _yPIDController.SetSetpoint(pose.Y().value());
  utils::WritePose2NT(nt::NetworkTableInstance::GetDefault().GetTable("swerveSetpoint"), GetSetpoint());
}

bool SwerveDrive::IsAtSetPose() {
  if (std::abs(_xPIDController.GetPositionError()) < 0.1 &&
      std::abs(_yPIDController.GetPositionError()) < 0.1) {
    if (std::abs(_xPIDController.GetVelocityError()) < 1 &&
        std::abs(_yPIDController.GetVelocityError()) < 1) {
      return false;
    }
  }
  return false;
}

bool SwerveDrive::IsAtSetAngle() {
  if (std::abs(_turnPIDController.GetPositionError()) < 0.1 &&
      std::abs(_turnPIDController.GetVelocityError()) < 0.1) {
    return true;
  }
  return false;
}

SwerveDriveState SwerveDrive::GetState() {
  return _state;
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
  #ifdef DEBUG
  std::cout << "RESETTING POSE" << std::endl;
  #endif
  _poseEstimator.ResetPosition(
      _config.gyro->GetRotation2d() /* - 1_rad*/,
      wpi::array<frc::SwerveModulePosition, 4>{_modules[3].GetPosition(), _modules[0].GetPosition(),
                                               _modules[1].GetPosition(), _modules[2].GetPosition()},
      frc::Pose2d(0_m, 0_m, 0_deg));
}

frc::Pose2d SwerveDrive::GetPose() {
  return frc::Pose2d(_poseEstimator.GetEstimatedPosition().X(), _poseEstimator.GetEstimatedPosition().Y(),
                     _poseEstimator.GetEstimatedPosition().Rotation());
  // return frc::Pose2d(1_m, 1_m, 0_deg);
}

void SwerveDrive::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
  // _poseEstimator.AddVisionMeasurement(pose, timestamp);
}

frc::Pose2d SwerveDrive::GetSetpoint() {
  return frc::Pose2d(units::meter_t{_xPIDController.GetSetpoint()},
                     units::meter_t{_yPIDController.GetSetpoint()},
                     units::radian_t{_anglePIDController.GetSetpoint()});
}

void SwerveDrive::MakeAtSetPoint() {
  ResetPose(GetSetpoint());
}

void SwerveDrive::TurnToAngle(units::radian_t angle) {
  _reqAngle = angle;
  _state = SwerveDriveState::kAngle;
}
}  // namespace drivetrain
}  // namespace wom
