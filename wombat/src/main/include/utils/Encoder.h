// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

namespace wom {
namespace utils {
class Encoder {
 public:
  Encoder(double encoderTicksPerRotation, int type, units::meter_t wheelRadius, double reduction = 1.0);

  virtual double GetEncoderRawTicks() const = 0;
  virtual double GetEncoderTickVelocity() const = 0;  // ticks/s
  virtual void ZeroEncoder();

  void SetEncoderPosition(units::degree_t position);
  void SetEncoderOffset(units::radian_t offset);

  double GetEncoderTicks() const;
  double GetEncoderTicksPerRotation() const;

  void SetReduction(double reduction);

  units::radian_t GetEncoderPosition();
  double GetEncoderDistance();
  units::radians_per_second_t GetEncoderAngularVelocity();  // rad/s

  int encoderType = 0;
  double _reduction;

  virtual double GetVelocity() const = 0;
  double GetVelocityValue() const;

  units::radian_t _offset = 0_rad;  // bad

 private:
  double _encoderTicksPerRotation;
  int _type = 0;
  units::meter_t _wheelRadius;
};

class DigitalEncoder : public Encoder {
 public:
  DigitalEncoder(int channelA, int channelB, double ticksPerRotation, units::meter_t wheelRadius,
                 double reduction = 1)
      : Encoder(ticksPerRotation, reduction, wheelRadius, 0), _nativeEncoder(channelA, channelB) {}

  double GetEncoderRawTicks() const override;
  double GetEncoderTickVelocity() const override;

  double GetPosition() const;
  double GetVelocity() const override;

 private:
  frc::Encoder _nativeEncoder;
};

class CANSparkMaxEncoder : public Encoder {
 public:
  explicit CANSparkMaxEncoder(rev::CANSparkMax* controller, units::meter_t wheelRadius, double reduction = 1);

  double GetEncoderRawTicks() const override;
  double GetEncoderTickVelocity() const override;

  double GetPosition() const;
  double GetVelocity() const override;

 protected:
  rev::SparkRelativeEncoder _encoder;
  friend class SimCANSparkMaxEncoder;
};

class TalonFXEncoder : public Encoder {
 public:
  TalonFXEncoder(ctre::phoenix6::hardware::TalonFX* controller, units::meter_t wheelRadius,
                 double reduction = 1);

  double GetEncoderRawTicks() const override;
  double GetEncoderTickVelocity() const override;
  double GetVelocity() const override;

 private:
  ctre::phoenix6::hardware::TalonFX* _controller;
};

class DutyCycleEncoder : public Encoder {
 public:
  DutyCycleEncoder(int channel, units::meter_t wheelRadius, double ticksPerRotation = 1,
                   double reduction = 1);

  double GetEncoderRawTicks() const override;
  double GetEncoderTickVelocity() const override;
  double GetVelocity() const override;

 private:
  frc::DutyCycleEncoder* _dutyCycleEncoder;
};

class CanEncoder : public Encoder {
 public:
  CanEncoder(int deviceNumber, units::meter_t wheelRadius, double ticksPerRotation = 4095,
             double reduction = 6.75, std::string name = "Drivebase");

  double GetEncoderRawTicks() const override;
  double GetEncoderTickVelocity() const override;
  double GetAbsoluteEncoderPosition();
  double GetVelocity() const override;

  const double constantValue = 0.0;

 private:
  ctre::phoenix6::hardware::CANcoder* _canEncoder;
};
}  // namespace utils
}  // namespace wom
