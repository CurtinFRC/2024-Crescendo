// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/Encoder.h"

#include <rev/SparkRelativeEncoder.h>

#include <cmath>

wom::utils::Encoder::Encoder(double encoderTicksPerRotation, int type, units::meter_t wheelRadius,
                             double reduction)
    : _reduction(reduction),
      _encoderTicksPerRotation(encoderTicksPerRotation),
      _type(type),
      _wheelRadius(wheelRadius) {}

double wom::utils::Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks();
}

double wom::utils::Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation * _reduction;
}

void wom::utils::Encoder::ZeroEncoder() {
  // _offtt = GetEncoderRawTicks() * 1_rad;
}

void wom::utils::Encoder::SetEncoderPosition(units::degree_t position) {
  // units::radian_t offset_turns = position - GetEncoderPosition();
  // units::degree_t offset = position - (GetEncoderRawTicks() * 360 * 1_deg);
  // _offset = offset;
  // _offset = -offset_turns;
}

void wom::utils::Encoder::SetEncoderOffset(units::radian_t offset) {  // HERE!
  _offset = offset;
  // units::turn_t offset_turns = offset;
  // _offset = offset_turns.value() * GetEncoderTicksPerRotation();
}

void wom::utils::Encoder::SetReduction(double reduction) {
  _reduction = reduction;
}

units::radian_t wom::utils::Encoder::GetEncoderPosition() {
  // if (_type == 0) {
  //   units::turn_t n_turns{GetEncoderTicks() / GetEncoderTicksPerRotation()};
  //   return n_turns;
  // } else if (_type == 2) {
  //   units::degree_t pos = GetEncoderTicks() * 1_deg;
  //   return pos;
  // } else {
  //   units::degree_t pos = GetEncoderTicks() * 1_deg;
  //   return pos - _offset;
  // }
  return GetEncoderTicks() * 1_rad;
}

double wom::utils::Encoder::GetEncoderDistance() {
  return GetEncoderTicks() * (2 * 3.1415) * _wheelRadius.value();
}

units::radians_per_second_t wom::utils::Encoder::GetEncoderAngularVelocity() {
  // return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2
  // * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() / GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

double wom::utils::Encoder::GetVelocityValue() const {
  // std::cout << "GET VELOCITY: " << GetVelocity() << std::endl;
  return GetVelocity();
  // return 0;
}

double wom::utils::DigitalEncoder::GetEncoderRawTicks() const {
  return _nativeEncoder.Get();
}

double wom::utils::DigitalEncoder::GetVelocity() const {
  return 0;
}

double wom::utils::DigitalEncoder::GetEncoderTickVelocity() const {
  return _nativeEncoder.GetRate();
}

wom::utils::CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax* controller, units::meter_t wheelRadius,
                                                   double reduction)
    : wom::utils::Encoder(42, reduction, wheelRadius, 2),
      _encoder(controller->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)) {}
double wom::utils::CANSparkMaxEncoder::GetEncoderRawTicks() const {
  return ((_encoder.GetPosition() * 2 * 3.1415) / 200);
}

double wom::utils::CANSparkMaxEncoder::GetEncoderTickVelocity() const {
  return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
}

double wom::utils::TalonFXEncoder::GetVelocity() const {
  return _controller->GetVelocity().GetValue().value();
}

double wom::utils::CanEncoder::GetVelocity() const {
  return _canEncoder->GetVelocity().GetValue().value();
}

double wom::utils::DutyCycleEncoder::GetVelocity() const {
  return 0;
}

double wom::utils::CANSparkMaxEncoder::GetPosition() const {
  return _encoder.GetPosition();
}

double wom::utils::CANSparkMaxEncoder::GetVelocity() const {
  return _encoder.GetVelocity();
}

wom::utils::TalonFXEncoder::TalonFXEncoder(ctre::phoenix6::hardware::TalonFX* controller,
                                           units::meter_t wheelRadius, double reduction)
    : utils::Encoder(2048, reduction, wheelRadius, 0), _controller(controller) {}

double wom::utils::TalonFXEncoder::GetEncoderRawTicks() const {
  return _controller->GetPosition().GetValue().value();
}

double wom::utils::TalonFXEncoder::GetEncoderTickVelocity() const {
  return _controller->Get() * 10;
}

wom::utils::DutyCycleEncoder::DutyCycleEncoder(int channel, units::meter_t wheelRadius,
                                               double ticksPerRotation, double reduction)
    : wom::utils::Encoder(ticksPerRotation, reduction, wheelRadius, 0) {
  _dutyCycleEncoder = new frc::DutyCycleEncoder(channel);
}

double wom::utils::DutyCycleEncoder::GetEncoderRawTicks() const {
  return _dutyCycleEncoder->GetAbsolutePosition();
}

double wom::utils::DutyCycleEncoder::GetEncoderTickVelocity() const {
  return 0;
}

wom::utils::CanEncoder::CanEncoder(int deviceNumber, units::meter_t wheelRadius, double ticksPerRotation,
                                   double reduction, std::string name)
    : wom::utils::Encoder(ticksPerRotation, 2, wheelRadius, reduction) {
  _canEncoder = new ctre::phoenix6::hardware::CANcoder(deviceNumber, name);
  // _canEncoder->ConfigAbsoluteEncoderRange(0, 1);
}

double wom::utils::CanEncoder::GetEncoderRawTicks() const {
  return (_canEncoder->GetAbsolutePosition().GetValue().value() * 2 * 3.14) - _offset.value();
}

double wom::utils::CanEncoder::GetEncoderTickVelocity() const {
  return _canEncoder->GetVelocity().GetValue().value();
}
