#include "ShooterBehaviour.h"


ShooterManualControl::ShooterManualControl(Shooter *shooter, frc::XboxController *codriver): _shooter(shooter), _codriver(codriver){
    Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {

    _shooter->table->GetEntry("RawControl").SetBoolean(_rawControl);

    if(_codriver->GetAButtonReleased()){
        _rawControl = !_rawControl;
    }

    if (!_rawControl) { 
        if (_codriver->GetLeftBumper()) {
            _shooter->SetState(ShooterState::kSpinUp);
            _shooter->SetPidGoal(65_rad_per_s);
        } else if (_codriver->GetRightBumper()) {
            _shooter->SetState(ShooterState::kSpinUp);
            _shooter->SetPidGoal(20_rad_per_s); 
        }
    } else {
         _shooter->SetRaw(_codriver->GetRightTriggerAxis() * 12_V);
         
    }

}
