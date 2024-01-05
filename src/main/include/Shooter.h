#pragma once

#include "Wombat.h"

struct ShooterConfig {
  std::string path;

  wom::Gearbox launcherGearBox;
  wom::Gearbox loaderGearBox;
};  

enum class LauncherState {
 kIdle,
 kLaunching,
 kSpinUp,
 kSpinDown,
 kInBattery,
 kRaw
};

enum class LoaderState {
  kIdle,
  kLoading,
  kReversing,
  kRaw
};

class Shooter : public behaviour::HasBehaviour {
 public:
  Shooter(ShooterConfig config);

  void OnUpdate(units::second_t dt);

  void setLauncherState(LauncherState state);
  void setLauncherRaw(units::volt_t voltage);
  void setLauncherRPM(units::revolutions_per_minute_t rpm);
  void setLoaderState(LoaderState state);
  void setLoaderRaw(units::volt_t voltage);
  void setLoaderRPM(units::revolutions_per_minute_t rpm);
  
 private:
  ShooterConfig _config;
  LauncherState _launcherState = LauncherState::kIdle;
  
  LoaderState _loaderState = LoaderState::kIdle;

  units::volt_t _launcherVoltage;
  units::volt_t _launcherRawVoltage;
  units::revolutions_per_minute_t _launcherRpm = 0_rpm;

  units::volt_t _loaderVoltage;
  units::volt_t _loaderRawVoltage;
  units::revolutions_per_minute_t _loaderRpm = 0_rpm;

  // wom::PIDController<units::revolutions_per_minute_t, units::volt_t> _launcherPID();

};