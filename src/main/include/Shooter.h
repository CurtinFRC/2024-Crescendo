#pragma once
#include "Wombat.h"
#include <units/angular_velocity.h>
#include <frc/DigitalInput.h>

struct ShooterConfig {
  std::string path;
  wom::Gearbox ShooterGearbox;
  wom::PIDConfig<units::radians_per_second, units::volt> pidConfig;
  frc::DigitalInput* shooterSensor;

};  

enum class ShooterState {
 kIdle,
 kShooting,
 kSpinUp,
 kReverse,
 kRaw
};


class Shooter : public behaviour::HasBehaviour{
  public:
    Shooter(ShooterConfig config);

    void OnUpdate(units::second_t dt);
    void setState(ShooterState state);
    void setRaw(units::volt_t voltage);
    void setPidGoal(units::radians_per_second_t);


    private:
    ShooterConfig _config;
    ShooterState _state = ShooterState::kIdle;
    units::volt_t _rawVoltage; 
    units::radians_per_second_t _goal;
    wom::PIDController<units::radians_per_second, units::volt> _pid;
    frc::DigitalInput _shooterSensor{0};

};
