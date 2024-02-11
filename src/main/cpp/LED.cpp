#include "LED.h"

LED::LED (){}



void LED::OnUpdate(units::second_t dt) {
  switch (_state)
  {
  case LEDState::kIdle: 
  {
    _led.frc::PWM::SetSpeed(0);
  }
  break;

  case LEDState::kAiming: 
  {
    _led.frc::PWM::SetSpeed(0.27);
  }
  break;
  case LEDState::kAmpReady: 
  {
    _led.frc::PWM::SetSpeed(0.87);
  }
  break;
  case LEDState::kHold: 
  {
    _led.frc::PWM::SetSpeed(0.57);
  }
  break;
  case LEDState::kIntaking: 
  {
    _led.frc::PWM::SetSpeed(0.07);
  }
  break;
  case LEDState::kShooterReady: 
  {
    _led.frc::PWM::SetSpeed(0.77);
  }
  break;
  
  default:
    break;
  }

}

void LED::SetState(LEDState state){
  _state = state;
}

LEDState LED::GetState(){
  return _state;
}