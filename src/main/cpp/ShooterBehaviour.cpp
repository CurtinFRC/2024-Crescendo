// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"

ShooterManualControl::ShooterManualControl(Shooter* shooter, frc::XboxController* tester)
    : _shooter(shooter), _tester(tester) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  table->GetEntry("RawControl").SetBoolean(_rawControl);


  if (_tester->GetAButtonReleased()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }

    if (_rawControl) {
      _shooter->SetState(ShooterState::kRaw);
      
      if (_tester->GetLeftTriggerAxis() > 0.1) {
        _shooter->SetRaw(12_V * _tester->GetLeftTriggerAxis());
      } else if (_tester->GetRightTriggerAxis() > 0.1) {
        _shooter->SetRaw(-12_V * _tester->GetRightTriggerAxis());
      } else {
        _shooter->SetRaw(0_V);

      }
      std::cout << "Raw" << std::endl;
    } else {
      _shooter->SetState(ShooterState::kSpinUp);
      _shooter->SetPidGoal(150_rad_per_s);
    }
  }
}
// void ShooterManualControl::OnTick(units::second_t dt) {
//   if (_codriver.GetAButtonPressed()) {
//     if (_rawControl == true) {
//       _rawControl = false;
//     } else {
//       _rawControl = true;
//     }
//     if (_rawControl) {
//     _shooter->setState(ShooterState::kRaw);
//     if (_codriver.GetLeftTriggerBumper()) {
//       _shooter->setRaw(12_V);
//     } else if (_codriver.GetRightBumper()) {
//       _shooter->setRaw(-12_V);
//     } else {
//       _shooter->setRaw(0_V);
//     }

//     // _shooter->setRaw(_codriver.GetLeftBumper() * 12_V);
//     // _shooter->setRaw(_codriver.GetRightBumper() * -12_V);
//     std::cout << "Raw" << std::endl;
//   }
//   }
// }

