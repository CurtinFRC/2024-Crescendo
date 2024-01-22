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
    void SetState(ShooterState state);
    void SetRaw(units::volt_t voltage);
    void SetPidGoal(units::radians_per_second_t);
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Shooter");
    ShooterConfig GetConfig() {return _config;}


    private:
    ShooterConfig _config;
    ShooterState _state = ShooterState::kRaw;
    units::volt_t _rawVoltage; 
    units::radians_per_second_t _goal;
    wom::PIDController<units::radians_per_second, units::volt> _pid;
    frc::DigitalInput _shooterSensor{0};

};
