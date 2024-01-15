#include "ShooterBehaviour.h"


ShooterManualControl::ShooterManualControl(Shooter *shooter, frc::XboxController &codriver): _shooter(shooter), _codriver(codriver){
    Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
    // if (_codriver.GetLeftBumper()) {
    //    _shooter->setState(ShooterState::kSpinUp);
    //    _shooter->setPidGoal(65_rad_per_s);
    // } else if (_codriver.GetRightBumper()) {
    //    _shooter->setState(ShooterState::kSpinUp);
    //    _shooter->setPidGoal(20_rad_per_s);
    // }
    double rawControl;
    if(_codriver.GetAButtonPressed()){
        if (rawControl == true){
            rawControl = false;
        } else {
            rawControl = true;
        }
    }
    if (rawControl == true) {
        if (_codriver.GetLeftBumper()) {
            _shooter->setState(ShooterState::kSpinUp);
            _shooter->setPidGoal(65_rad_per_s);
        } else if (_codriver.GetRightBumper()) {
            _shooter->setState(ShooterState::kSpinUp);
            _shooter->setPidGoal(20_rad_per_s);
    }
    }
    //yay now to face... BUILD ERRORS....
    // "im not sure..." - one of the members
    // We should get the info from arm class to influence the shooter.
    // if the arm state is in speaker position.. do this... if the arm state is in the amp position then do this...
    // We want to detect the amount that one of the buttons is pressed in and based off of that we can increaseordecrease speed.
}


