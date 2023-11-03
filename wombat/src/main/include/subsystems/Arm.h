#pragma once

#include "behaviour/HasBehaviour.h"
#include "utils/Encoder.h"
#include "utils/Gearbox.h"
#include "utils/PID.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <units/mass.h>
#include <units/voltage.h>
#include <units/current.h>

namespace wom {
  struct ArmConfig {
    std::string path;

    wom::utils::Gearbox leftGearbox;
    wom::utils::Gearbox rightGearbox;
    rev::SparkMaxRelativeEncoder armEncoder;
    wom::utils::PIDConfig<units::radian, units::volt> pidConfig;
    wom::utils::PIDConfig<units::radians_per_second, units::volt> velocityConfig;

    units::kilogram_t armMass;
    units::kilogram_t loadMass;
    units::meter_t armLength;
    units::radian_t minAngle = 0_deg;
    units::radian_t maxAngle = 180_deg;
    units::radian_t initialAngle = 0_deg;
    units::radian_t angleOffset = 0_deg;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  enum class ArmState {
    kIdle,
    kAngle,
    kRaw,
    kVelocity
  };

  class Arm : public behaviour::HasBehaviour {
  public:
    Arm(ArmConfig config);

    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetAngle(units::radian_t angle);
    void SetRaw(units::volt_t voltage);
    void SetVelocity(units::radians_per_second_t velocity);

    void SetArmSpeedLimit(double limit); //units, what are they?? 

    ArmConfig &GetConfig();

    units::radian_t GetAngle() const;
    units::radians_per_second_t MaxSpeed() const;
    units::radians_per_second_t GetArmVelocity() const;
    
    bool IsStable() const;
  private:
    ArmConfig _config;
    ArmState _state = ArmState::kIdle;
    wom::utils::PIDController<units::radian, units::volt> _pid;
    wom::utils::PIDController<units::radians_per_second, units::volt> _velocityPID;
    
    std::shared_ptr<nt::NetworkTable> _table;

    double armLimit = 0.4;
    units::radians_per_second_t lastVelocity;

    units::volt_t _voltage{0};
  };
};
