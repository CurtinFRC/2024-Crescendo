#pragma once

#include <frc/Encoder.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/DutyCycleEncoder.h>

#include <iostream>

#include "utils/Util.h"

namespace wom {
namespace utils {
  class Encoder {
   public:
    Encoder(double encoderTicksPerRotation, double reduction, int type);
    virtual double    GetEncoderRawTicks() const = 0;
    virtual double    GetEncoderTickVelocity() const = 0;  // ticks/s
    virtual void      ZeroEncoder();

    void SetEncoderPosition(units::degree_t position);
    void SetEncoderOffset(units::radian_t offset);

    double  GetEncoderTicks() const;
    double  GetEncoderTicksPerRotation() const;

    void SetReduction(double reduction);

    units::radian_t GetEncoderPosition();
    double GetEncoderDistance();
    units::radians_per_second_t GetEncoderAngularVelocity();   // rad/s

    int encoderType = 0;
    double _reduction = 1.0;
   private:
    double _encoderTicksPerRotation;
    int _type = 0;
    units::radian_t _offset = 0_rad;
  };

  class DigitalEncoder : public Encoder {
   public:
    DigitalEncoder(int channelA, int channelB, double ticksPerRotation, double reduction = 1)
        : Encoder(ticksPerRotation, reduction, 0),
          _nativeEncoder(channelA, channelB){};

    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;
   private:
    frc::Encoder _nativeEncoder;
  };

  class SimCANSparkMaxEncoder;
  class CANSparkMaxEncoder : public Encoder {
   public:
    CANSparkMaxEncoder(rev::CANSparkMax *controller, double reduction = 1);

    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;
    
    double GetPosition() const;
    double GetVelocity() const;

   protected:
    rev::SparkMaxRelativeEncoder _encoder;
    friend class SimCANSparkMaxEncoder;
  };

  class TalonFXEncoder : public Encoder {
   public:
    TalonFXEncoder(ctre::phoenix::motorcontrol::can::TalonFX *controller, double reduction = 1);

    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;

   private:
    ctre::phoenix::motorcontrol::can::TalonFX *_controller;
  };

  class TalonSRXEncoder : public Encoder {
   public: 
    TalonSRXEncoder(ctre::phoenix::motorcontrol::can::TalonSRX *controller, double ticksPerRotation, double reduction = 1);
   
    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;

   private: 
    ctre::phoenix::motorcontrol::can::TalonSRX *_controller;
  };

  class DutyCycleEncoder : public Encoder {
   public: 
    DutyCycleEncoder(int channel, double ticksPerRotation = 1, double reduction = 1);

    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;

   private: 
    frc::DutyCycleEncoder _dutyCycleEncoder;
  };

  class CanEncoder : public Encoder {
    public: 
      CanEncoder(int deviceNumber, double ticksPerRotation = 4095, double reduction = 1);

      double GetEncoderRawTicks() const override;
      double GetEncoderTickVelocity() const override;
      double GetAbsoluteEncoderPosition();

      const double constantValue = 0.0;

    private: 
      CANCoder *_canEncoder;
  };
} // namespace utils
}  // namespace wom
