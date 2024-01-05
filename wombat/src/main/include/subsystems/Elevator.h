// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <networktables/NetworkTable.h>
#include <units/length.h>
#include <units/mass.h>

#include <memory>
#include <string>

#include "behaviour/HasBehaviour.h"
#include "utils/Gearbox.h"
#include "utils/PID.h"

namespace wom {
namespace subsystems {
  enum class ElevatorState { kIdle, kPID, kManual, kVelocity };

  struct ElevatorConfig {
    std::string                                                  path;
    wom::utils::Gearbox                                          leftGearbox;
    wom::utils::Gearbox                                          rightGearbox;
    rev::SparkMaxRelativeEncoder                                 elevatorEncoder;
    frc::DigitalInput                                           *topSensor;
    frc::DigitalInput                                           *bottomSensor;
    units::meter_t                                               radius;
    units::kilogram_t                                            mass;
    units::meter_t                                               maxHeight;
    units::meter_t                                               minHeight;
    units::meter_t                                               initialHeight;
    wom::utils::PIDConfig<units::meter, units::volt>             pid;
    wom::utils::PIDConfig<units::meters_per_second, units::volt> velocityPID;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  class Elevator : public behaviour::HasBehaviour {
   public:
    explicit Elevator(ElevatorConfig params);

    void OnUpdate(units::second_t dt);

    void SetManual(units::volt_t voltage);
    void SetPID(units::meter_t height);

    void SetState(ElevatorState state);

    void SetVelocity(units::meters_per_second_t velocity);

    units::volt_t GetRaw();

    double GetElevatorEncoderPos();
    void   SetElevatorSpeedLimit(double limit);

    ElevatorConfig &GetConfig();

    bool          IsStable() const;
    ElevatorState GetState() const;

    units::meter_t             GetHeight() const;
    units::meters_per_second_t MaxSpeed() const;
    units::meters_per_second_t GetElevatorVelocity() const;

   private:
    units::volt_t _setpointManual{0};

    ElevatorConfig _config;
    ElevatorState  _state;
    double         speedLimit = 0.5;

    units::meters_per_second_t _velocity;

    wom::utils::PIDController<units::meter, units::volt>             _pid;
    wom::utils::PIDController<units::meters_per_second, units::volt> _velocityPID;

    std::shared_ptr<nt::NetworkTable> _table;
  };
}  // namespace subsystems
}  // namespace wom
