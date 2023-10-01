#pragma once 

#include "utils/Gearbox.h"
#include "PID.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <units/length.h>
#include <units/mass.h>

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <networktables/NetworkTable.h>

#include <memory>

namespace wom {
  enum class ElevatorState {
    kIdle, 
    kPID,
    kManual,
    kVelocity
  };

  struct ElevatorConfig {
    std::string path;
    wom::Gearbox leftGearbox;
    wom::Gearbox rightGearbox;
    rev::SparkMaxRelativeEncoder elevatorEncoder;
    frc::DigitalInput *topSensor;
    frc::DigitalInput *bottomSensor;
    units::meter_t radius;
    units::kilogram_t mass;
    units::meter_t maxHeight;
    units::meter_t minHeight;
    units::meter_t initialHeight;
    PIDConfig<units::meter, units::volt> pid;
    PIDConfig<units::meters_per_second, units::volt> velocityPID;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  class Elevator : public behaviour::HasBehaviour {
   public: 
    Elevator(ElevatorConfig params);

    void OnUpdate(units::second_t dt);

    void SetManual(units::volt_t voltage);
    void SetPID(units::meter_t height);
    void SetIdle();
    void SetRaw();

    void SetVelocity(units::meters_per_second_t velocity);

    units::volt_t GetRaw();

    double GetElevatorEncoderPos();
    void SetElevatorSpeedLimit(double limit);

    ElevatorConfig &GetConfig();
    
    bool IsStable() const;
    ElevatorState GetState() const;

    units::meter_t GetHeight() const;
    units::meters_per_second_t MaxSpeed() const;
    units::meters_per_second_t GetElevatorVelocity() const;
  
   private:
    units::volt_t _setpointManual{0};

    ElevatorConfig _config;
    ElevatorState _state;
    double speedLimit = 0.5;

    units::meters_per_second_t _velocity;

    PIDController<units::meter, units::volt> _pid;
    PIDController<units::meters_per_second, units::volt> _velocityPID;

    std::shared_ptr<nt::NetworkTable> _table;
  };
};
