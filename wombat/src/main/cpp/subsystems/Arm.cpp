#include "subsystems/Arm.h"

#include <units/math.h>

using namespace frc;

//creates network table instatnce on shuffleboard
void wom::subsystems::ArmConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) {
  table->GetEntry("armMass").SetDouble(armMass.value());
  table->GetEntry("loadMass").SetDouble(loadMass.value());
  table->GetEntry("armLength").SetDouble(armLength.value());
  table->GetEntry("minAngle").SetDouble(minAngle.convert<units::degree>().value());
  table->GetEntry("maxAngle").SetDouble(maxAngle.convert<units::degree>().value());
  table->GetEntry("initialAngle").SetDouble(initialAngle.convert<units::degree>().value());
  table->GetEntry("angleOffset").SetDouble(initialAngle.convert<units::degree>().value());
}

//arm config is used
wom::subsystems::Arm::Arm(wom::subsystems::ArmConfig config)
  : _config(config),
    _pid(config.path + "/pid", config.pidConfig),
    _velocityPID(config.path + "/velocityPID", config.velocityConfig),
    _table(nt::NetworkTableInstance::GetDefault().GetTable(config.path))
{
}

//the loop that allows the information to be used
void wom::subsystems::Arm::OnUpdate(units::second_t dt) {
  //sets the voltage and gets the current angle
  units::volt_t voltage = 0_V;
  auto angle = GetAngle();

  //sets usable infromation for each state
  switch (_state) {
    case wom::subsystems::ArmState::kIdle:
      break;
    case wom::subsystems::ArmState::kVelocity:
      {
        units::newton_meter_t torque = 9.81_m / 1_s / 1_s * _config.armLength * units::math::cos(angle + _config.angleOffset) * (0.5 * _config.armMass + _config.loadMass);
        // units::volt_t feedforward = _config.leftGearbox.motor.Voltage(torque, 0_rad/1_s);
        units::volt_t feedforward = _config.leftGearbox.motor.Voltage(torque, _velocityPID.GetSetpoint());
        // feedforward = 3.5_V;
        // std::cout << "feedforward" << feedforward.value() << std::endl;
        voltage = _velocityPID.Calculate(GetArmVelocity(), dt, feedforward);
        // std::cout << "arm velocity voltage is: " << voltage.value() << std::endl;
        // voltage = 0_V;
      }
      break;
    case wom::subsystems::ArmState::kAngle:
      {
        units::newton_meter_t torque = 9.81_m / 1_s / 1_s * _config.armLength * units::math::cos(angle + _config.angleOffset) * (0.5 * _config.armMass + _config.loadMass);
        units::volt_t feedforward = _config.leftGearbox.motor.Voltage(torque, 0_rad/ 1_s);
        // std::cout << "feedforward" << feedforward.value() << std::endl;
        voltage = _pid.Calculate(angle, dt, feedforward);
      }
      break;
    case wom::subsystems::ArmState::kRaw:
      voltage = _voltage;
      break;
  }

  // if (
  //   (((_config.minAngle + _config.angleOffset) < 75_deg && units::math::abs(_pid.GetSetpoint() - _config.minAngle) <= 1_deg)
    //  || ((_config.maxAngle + _config.angleOffset) > 105_deg && units::math::abs(_pid.GetSetpoint() - _config.maxAngle) <= 1_deg)) && 
  //   units::math::abs(_pid.GetError()) <= 1_deg
  // ) {
  //   voltage = 0_V;
  // }

  voltage *= armLimit;

  // units::newton_meter_t torqueLimit = 10_kg * 1.4_m * 6_mps_sq;
  // units::volt_t voltageMax = _config.leftGearbox.motor.Voltage(torqueLimit, _config.leftGearbox.encoder->GetEncoderAngularVelocity());
  // units::volt_t voltageMin = _config.leftGearbox.motor.Voltage(-torqueLimit, _config.leftGearbox.encoder->GetEncoderAngularVelocity());

  // voltage = units::math::max(units::math::min(voltage, voltageMax), voltageMin);
  units::volt_t voltageMin = -5.5_V;
  units::volt_t voltageMax = 5.5_V;
  voltage = units::math::max(units::math::min(voltage, voltageMax), voltageMin);

  // std::cout << "voltage: " << voltage.value() << std::endl;

  _config.leftGearbox.transmission->SetVoltage(voltage);
  _config.rightGearbox.transmission->SetVoltage(voltage);

  //creates network table instances for the angle and config of the arm
  _table->GetEntry("angle").SetDouble(angle.convert<units::degree>().value());
  _config.WriteNT(_table->GetSubTable("config"));
}

void wom::subsystems::Arm::SetArmSpeedLimit(double limit) {
  armLimit = limit;
}

//defines information needed for the functions and connects the states to their respective function

void wom::subsystems::Arm::SetIdle() {
  _state = ArmState::kIdle;
}

void wom::subsystems::Arm::SetRaw(units::volt_t voltage) {
  _state = wom::subsystems::ArmState::kRaw;
  _voltage = voltage;
}

void wom::subsystems::Arm::SetAngle(units::radian_t angle) {
  _state = wom::subsystems::ArmState::kAngle;
  _pid.SetSetpoint(angle);
}

void wom::subsystems::Arm::SetVelocity(units::radians_per_second_t velocity) {
  _state = wom::subsystems::ArmState::kVelocity;
  _velocityPID.SetSetpoint(velocity);
}

wom::subsystems::ArmConfig &wom::subsystems::Arm::GetConfig() {
  return _config;
}

units::radian_t wom::subsystems::Arm::GetAngle() const {
  return _config.armEncoder.GetPosition() / 100 * 360 * 1_deg;
}

units::radians_per_second_t wom::subsystems::Arm::MaxSpeed() const {
  return _config.leftGearbox.motor.Speed(0_Nm, 12_V);
}

units::radians_per_second_t wom::subsystems::Arm::GetArmVelocity() const {
  return _config.armEncoder.GetVelocity() / 100 * 360 * 1_deg / 60_s;
}

bool wom::subsystems::Arm::IsStable() const {
  return _pid.IsStable(5_deg);
}


/* SIMULATION */
// #include <units/math.h>

// ::wom::sim::ArmSim::ArmSim(ArmConfig config) 
//   : config(config),
//     angle(config.initialAngle),
//     encoder(config.gearbox.encoder->MakeSimEncoder()),
//     lowerLimit(config.lowerLimitSwitch ? new frc::sim::DIOSim(*config.lowerLimitSwitch) : nullptr),
//     upperLimit(config.upperLimitSwitch ? new frc::sim::DIOSim(*config.upperLimitSwitch) : nullptr),
//     table(nt::NetworkTableInstance::GetDefault().GetTable(config.path + "/sim"))
//   {}

// units::ampere_t wom::sim::ArmSim::GetCurrent() const {
//   return current;
// }

// void ::wom::sim::ArmSim::Update(units::second_t dt) {
//   torque = (config.loadMass * config.armLength + config.armMass * config.armLength / 2.0) * 9.81_m / 1_s / 1_s * units::math::cos(config.angleOffset + angle) + additionalTorque;
//   velocity = config.gearbox.motor.Speed(torque, config.gearbox.transmission->GetEstimatedRealVoltage());
//   angle += velocity * dt;

//   if (angle <= config.minAngle) {
//     angle = config.minAngle;
//     velocity = 0_rad / 1_s;
//     if (lowerLimit) lowerLimit->SetValue(true);
//   } else {
//     if (lowerLimit) lowerLimit->SetValue(false);
//   }

//   if (angle >= config.maxAngle) {
//     angle = config.maxAngle;
//     velocity = 0_rad / 1_s;
//     if (upperLimit) upperLimit->SetValue(true);
//   } else {
//     if (upperLimit) upperLimit->SetValue(false);
//   }

//   current = config.gearbox.motor.Current(velocity, config.gearbox.transmission->GetEstimatedRealVoltage());

//   if (encoder) encoder->SetEncoderTurns(angle - config.initialAngle);

//   table->GetEntry("angle").SetDouble(angle.convert<units::degree>().value());
//   table->GetEntry("current").SetDouble(current.value());
// }
