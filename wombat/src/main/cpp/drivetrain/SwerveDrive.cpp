// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/SwerveDrive.h"

wom::drivetrain::SwerveModule::SwerveModule(wom::drivetrain::SwerveModuleConfig config,
                                            wom::drivetrain::SwerveModuleState  state)
    : _rotationalVelocityPID(config.path + "/pid/rotationalVelocity", config.rotationalVelocityPID),
      _movementVelocityPID(config.path + "/pid/movementVelocity", config.movementVelocityPID),
      _rotationalPID(config.path + "/pid/rotationGearbox", config.rotationPID),
      _movementPID(config.path + "/pid/movementGearbox", config.movementPID),
      _config(config),
      _state(state) {
  // TODO! Get name the table based off module name
  table = nt::NetworkTableInstance::GetDefault().GetTable("Swerve Module");
}

wom::drivetrain::SwerveModuleConfig wom::drivetrain::SwerveModule::GetConfig() {
  return _config;
}

wom::drivetrain::SwerveModuleState wom::drivetrain::SwerveModule::GetState() {
  return _state;
}

void wom::drivetrain::SwerveModule::SetState(wom::drivetrain::SwerveModuleState state) {
  _state = state;
}

void wom::drivetrain::SwerveModule::OnStart(units::radian_t offset) {
  switch (_config.name) {
    case wom::drivetrain::SwerveModuleName::FrontLeft:
      name = "Front Left";
      break;
    case wom::drivetrain::SwerveModuleName::FrontRight:
      name = "Front Right";
      break;
    case wom::drivetrain::SwerveModuleName::BackLeft:
      name = "Back Left";
      break;
    case wom::drivetrain::SwerveModuleName::BackRight:
      name = "Back Right";
      break;
    default:
      name = "Invalid Name";
      std::cout << "Invalid Name" << std::endl;
      break;
  }

  std::cout << "Starting Swerve Module" << std::endl;
  std::cout << "Module name: " << name << std::endl;
  _config.rotationGearbox.encoder->SetEncoderPosition(offset);
  _config.movementGearbox.encoder->SetEncoderPosition(offset);
}

void wom::drivetrain::SwerveModule::PIDControl(units::second_t dt, units::radian_t rotation,
                                               units::meter_t movement) {
  units::volt_t feedforwardRotationalVelocity = _config.rotationGearbox.motor.Voltage(
      0_Nm, _config.rotationGearbox.encoder->GetEncoderAngularVelocity());
  voltageRotation = _rotationalVelocityPID.Calculate(angularVelocity, dt, feedforwardRotationalVelocity);
  if (voltageRotation > 11_V) {
    voltageRotation = 11_V;
  }
  _config.rotationGearbox.transmission->SetVoltage(voltageRotation);

  units::volt_t feedforwardMovementVelocity = _config.movementGearbox.motor.Voltage(
      0_Nm, _config.movementGearbox.encoder->GetEncoderAngularVelocity());
  voltageMovement = _movementVelocityPID.Calculate(velocity, dt, feedforwardMovementVelocity);
  if (voltageMovement > 12_V) {
    voltageMovement = 12_V;
  }
  _config.movementGearbox.transmission->SetVoltage(voltageMovement);
}

units::meters_per_second_t wom::drivetrain::SwerveModule::GetSpeed() {
  return units::meters_per_second_t{_config.movementGearbox.encoder->GetEncoderAngularVelocity().value() *
                                    _config.wheelRadius.value()};
}

void wom::drivetrain::SwerveModule::Log() {
  table->GetEntry("Velocity").SetDouble(velocity.value());
  table->GetEntry("Angular Velocity").SetDouble(angularVelocity.value());
  table->GetEntry("Movement Voltage").SetDouble(voltageMovement.value());
  table->GetEntry("Angular Voltage").SetDouble(voltageRotation.value());
}

void wom::drivetrain::SwerveModule::OnUpdate(units::second_t dt, units::radian_t rotation,
                                             units::meter_t movement, units::volt_t rotationVoltage) {
  Log();

  switch (_state) {
    case wom::drivetrain::SwerveModuleState::kIdle:
      break;
    case wom::drivetrain::SwerveModuleState::kPID:
      PIDControl(dt, rotation, movement);
      break;
    case wom::drivetrain::SwerveModuleState::kCalibration:
      PIDControl(dt, units::radian_t{180}, units::meter_t{0});
      break;
    case wom::drivetrain::SwerveModuleState::kManualTurn:
      _config.rotationGearbox.transmission->SetVoltage(rotationVoltage);
      break;
    default:
      std::cout << "Invalid State" << std::endl;
      break;
  }
}

wom::drivetrain::Swerve::Swerve(wom::drivetrain::SwerveConfig config, wom::drivetrain::SwerveState state,
                                wom::vision::Limelight *vision)
    : _config(config), _state(state), _vision(vision) {}

wom::drivetrain::Swerve::Swerve(wom::drivetrain::SwerveConfig config, wom::drivetrain::SwerveState state,
                                Pigeon2 *gyro)
    : _config(config), _state(state), _gyro(gyro) {}

wom::drivetrain::SwerveConfig wom::drivetrain::Swerve::GetConfig() {
  return _config;
}

wom::drivetrain::SwerveState wom::drivetrain::Swerve::GetState() {
  return _state;
}

Pigeon2 *wom::drivetrain::Swerve::GetGyro() {
  return _gyro;
}

void wom::drivetrain::Swerve::SetState(wom::drivetrain::SwerveState state) {
  _state = state;
}

void wom::drivetrain::Swerve::FieldRelativeControl(frc::Pose3d desiredPose, units::second_t dt) {
  units::meter_t  movement = units::math::hypot(desiredPose.X(), desiredPose.Y());
  units::radian_t rotation = units::math::acos(movement / desiredPose.X());

  if (rotation > 0_rad) {
    rotation += 45_rad;
  } else {
    rotation -= 45_rad;
  }
  _config.frontLeft.SetState(wom::drivetrain::SwerveModuleState::kPID);
  _config.frontLeft.OnUpdate(dt, rotation, movement, 0_V);
  _config.frontRight.SetState(wom::drivetrain::SwerveModuleState::kPID);
  _config.frontRight.OnUpdate(dt, rotation, movement, 0_V);

  if (rotation > 0_rad) {
    rotation -= 90_rad;
  } else {
    rotation += 90_rad;
  }
  _config.backLeft.SetState(wom::drivetrain::SwerveModuleState::kPID);
  _config.backLeft.OnUpdate(dt, rotation, movement, 0_V);
  _config.backRight.SetState(wom::drivetrain::SwerveModuleState::kPID);
  _config.backRight.OnUpdate(dt, rotation, movement, 0_V);
}

void wom::drivetrain::Swerve::OnStart() {
  _config.backLeft.OnStart(0_rad);
  _config.backRight.OnStart(0_rad);
  _config.frontLeft.OnStart(0_rad);
  _config.frontRight.OnStart(0_rad);
  std::cout << "Starting Serve" << std::endl;
}

void wom::drivetrain::Swerve::OnUpdate(units::second_t dt, wom::vision::Limelight *vision,
                                       frc::Pose3d desiredPose) {
  vision->OnUpdate(dt);

  switch (_state) {
    case wom::drivetrain::SwerveState::kIdle:
      break;
    case wom::drivetrain::SwerveState::kPose:
      break;
    case wom::drivetrain::SwerveState::kFieldRelative:
      FieldRelativeControl(desiredPose, dt);
      break;
    default:
      std::cout << "Invalid State" << std::endl;
      break;
  }
}

void wom::drivetrain::Swerve::RobotRelativeControl(units::second_t dt, units::radian_t desiredDirection,
                                                   units::meter_t magnitude) {}

void wom::drivetrain::Swerve::OnUpdate(units::second_t dt, Pigeon2 *gyro) {
  switch (_state) {
    case wom::drivetrain::SwerveState::kIdle:
      break;
    case wom::drivetrain::SwerveState::kPose:
      break;
    case wom::drivetrain::SwerveState::kRobotRelative:
      break;
    default:
      std::cout << "Invalid State" << std::endl;
      break;
  }
}

wom::vision::Limelight *wom::drivetrain::Swerve::GetLimelight() {
  return _vision;
}

