#include "drivetrain/SwerveDrive.h"

wom::drivetrain::SwerveModule::SwerveModule(wom::drivetrain::SwerveModuleConfig config, wom::drivetrain::SwerveModuleState state) :
  _rotationalVelocityPID(config.path + "/pid/rotationalVelocity", config.rotationalVelocityPID),
  _movementVelocityPID(config.path + "/pid/movementVelocity", config.movementVelocityPID),
  _rotationalPID(config.path + "/pid/rotationGearbox", config.rotationPID),
  _movementPID(config.path + "/pid/movementGearbox", config.movementPID),
  _config(config),
  _state(state)
{
  angularVelocityPublisher = _config.angularVelocityTopic.Publish();
  velocityPublisher = _config.velocityTopic.Publish();
}

wom::drivetrain::SwerveModuleConfig wom::drivetrain::SwerveModule::GetConfig() {
  return _config;
}

wom::drivetrain::SwerveModuleState wom::drivetrain::SwerveModule::GetState() {
  return _state;
}

void wom::drivetrain::SwerveModule::SetState(wom::drivetrain::SwerveModuleState state) { _state = state; }

void wom::drivetrain::SwerveModule::OnStart() {
  std::string name;
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
      std::cout << "Invalid Name" << std::endl;;
      break;
  }

  std::cout << "Starting Swerve Module" << std::endl;
  std::cout << "Module name: " << name << std::endl;
}

void wom::drivetrain::SwerveModule::PIDControl(units::second_t dt, units::radian_t rotation, units::meter_t movement) {

  units::volt_t feedforwardRotationalVelocity = _config.rotationGearbox.motor.Voltage(0_Nm, _config.rotationGearbox.encoder->GetEncoderAngularVelocity());
  voltageRotation = _rotationalVelocityPID.Calculate(angularVelocity, dt, feedforwardRotationalVelocity);
  _config.rotationGearbox.transmission->SetVoltage(voltageRotation);

  units::volt_t feedforwardMovementVelocity = _config.movementGearbox.motor.Voltage(0_Nm, _config.movementGearbox.encoder->GetEncoderAngularVelocity());
  voltageMovement = _movementVelocityPID.Calculate(velocity, dt, feedforwardMovementVelocity);
  _config.movementGearbox.transmission->SetVoltage(voltageMovement);
}

units::meters_per_second_t wom::drivetrain::SwerveModule::GetSpeed() {
  return units::meters_per_second_t{_config.movementGearbox.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
}

void wom::drivetrain::SwerveModule::Log() {
    int64_t time = nt::Now();
    velocityPublisher.Set(GetSpeed().value(), time);
    angularVelocityPublisher.Set(_config.rotationGearbox.encoder->GetEncoderAngularVelocity().value(), time);
}

void wom::drivetrain::SwerveModule::OnUpdate(units::second_t dt, units::radian_t rotation, units::meter_t movement) {
  _config.vision.OnUpdate(dt);
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
    default:
      std::cout << "Invalid State" << std::endl;
      break;
  }
}

