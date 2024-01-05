#pragma once
#include "wombat.h"
struct IntakeConfig{
    wom::Gearbox intakeGearbox;
};
enum class IntakeState {
    kIntaking,
    kOutaking,
    kIdle
};

class Intake : public behaviour::HasBehaviour{
  public:
    Intake(IntakeConfig config);

    void OnUpdate(units::second_t dt);
    void setState(IntakeState state);
    void setRaw(units::volt_t voltage);
    private:
    IntakeConfig _config;
    IntakeState _state = IntakeState::kIdle;
    units::volt_t _voltage;

};