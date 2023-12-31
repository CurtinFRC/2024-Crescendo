// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/Encoder.h"

double wom::utils::Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks();
}

double wom::utils::Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation * _reduction;
}

void wom::utils::Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks() * 1_rad;
}

void wom::utils::Encoder::SetEncoderPosition(units::degree_t position) {
  // units::radian_t offset_turns = position - GetEncoderPosition();
  units::degree_t offset = position - (GetEncoderRawTicks() * 360 * 1_deg);
  _offset = offset;
  // _offset = -offset_turns;
}

void wom::utils::Encoder::SetEncoderOffset(units::radian_t offset) {
  _offset = offset;
  // units::turn_t offset_turns = offset;
  // _offset = offset_turns.value() * GetEncoderTicksPerRotation();
}

void wom::utils::Encoder::SetReduction(double reduction) {
  _reduction = reduction;
}

units::radian_t wom::utils::Encoder::GetEncoderPosition() {
  if (_type == 0) {
    units::turn_t n_turns{GetEncoderTicks() / GetEncoderTicksPerRotation()};
    return n_turns;
  } else if (_type == 2) {
    units::degree_t pos = GetEncoderTicks() * 1_deg;
    return pos - _offset;
  } else {
    units::degree_t pos = GetEncoderTicks() * 1_deg;
    return pos - _offset;
  }
}

double wom::utils::Encoder::GetEncoderDistance() {
  return (GetEncoderTicks() /*- _offset.value()*/) * 0.02032;
  // return (GetEncoderTicks() - _offset.value()) * 2 * 3.141592 * 0.0444754;
  // return (GetEncoderTicks() - _offset.value());
}

units::radians_per_second_t wom::utils::Encoder::GetEncoderAngularVelocity() {
  // return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2
  // * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() /
                                          GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

double wom::utils::DigitalEncoder::GetEncoderRawTicks() const {
  return _nativeEncoder.Get();
}

double wom::utils::DigitalEncoder::GetEncoderTickVelocity() const {
  return _nativeEncoder.GetRate();
}

wom::utils::CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax* controller,
                                                   double reduction)
    : wom::utils::Encoder(42, reduction, 2),
      _encoder(controller->GetEncoder()) {}

double wom::utils::CANSparkMaxEncoder::GetEncoderRawTicks() const {
  return _encoder.GetPosition() * _reduction;
}

double wom::utils::CANSparkMaxEncoder::GetEncoderTickVelocity() const {
  return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
}

wom::utils::TalonFXEncoder::TalonFXEncoder(
    ctre::phoenix::motorcontrol::can::TalonFX* controller, double reduction)
    : wom::utils::Encoder(2048, reduction, 0), _controller(controller) {
  controller->ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::TalonFXFeedbackDevice::IntegratedSensor);
}

double wom::utils::TalonFXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double wom::utils::TalonFXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

wom::utils::TalonSRXEncoder::TalonSRXEncoder(
    ctre::phoenix::motorcontrol::can::TalonSRX* controller,
    double ticksPerRotation, double reduction)
    : wom::utils::Encoder(ticksPerRotation, reduction, 0),
      _controller(controller) {
  controller->ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::QuadEncoder);
}

double wom::utils::TalonSRXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double wom::utils::TalonSRXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

wom::utils::DutyCycleEncoder::DutyCycleEncoder(int channel,
                                               double ticksPerRotation,
                                               double reduction)
    : wom::utils::Encoder(ticksPerRotation, reduction, 0),
      _dutyCycleEncoder(channel) {}

double wom::utils::DutyCycleEncoder::GetEncoderRawTicks() const {
  return _dutyCycleEncoder.Get().value();
}

double wom::utils::DutyCycleEncoder::GetEncoderTickVelocity() const {
  return 0;
}

wom::utils::CanEncoder::CanEncoder(int deviceNumber, double ticksPerRotation,
                                   double reduction)
    : wom::utils::Encoder(ticksPerRotation, reduction, 1) {
  _canEncoder = new CANCoder(deviceNumber, "Drivebase");
}

double wom::utils::CanEncoder::GetEncoderRawTicks() const {
  return _canEncoder->GetAbsolutePosition();
}

double wom::utils::CanEncoder::GetEncoderTickVelocity() const {
  return _canEncoder->GetVelocity();
}
