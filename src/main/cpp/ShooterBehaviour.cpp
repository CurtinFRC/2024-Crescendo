// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"
#include "Shooter.h"
#include "utils/Util.h"

class Vision;

ShooterManualControl::ShooterManualControl(Shooter* shooter, frc::XboxController* tester, LED* led)
    : _shooter(shooter), _codriver(tester), _led(led) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  table->GetEntry("RawControl").SetBoolean(_rawControl);

  if (_codriver->GetBackButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    _shooter->SetState(ShooterState::kRaw);
    if (_codriver->GetLeftTriggerAxis() > 0.1) {
      _shooter->SetRaw(12_V * _codriver->GetLeftTriggerAxis());
    } else if (_codriver->GetRightTriggerAxis() > 0.1) {
      _shooter->SetRaw(-12_V * _codriver->GetRightTriggerAxis());
    } else {
      _shooter->SetRaw(0_V);
    }
  } else {
    if (_codriver->GetXButton()) {
      _shooter->SetPidGoal(150_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
      _led->SetState(LEDState::kAiming);
    } else if (_codriver->GetYButton()) {
      _shooter->SetPidGoal(300_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
      _led->SetState(LEDState::kAiming);
    } else if (_codriver->GetLeftTriggerAxis() > 0.1) {
      _shooter->SetState(ShooterState::kRaw);
      _led->SetState(LEDState::kIdle);
      _shooter->SetRaw(12_V * _codriver->GetLeftTriggerAxis());
    } else if (_codriver->GetRightTriggerAxis() > 0.1) {
      _shooter->SetState(ShooterState::kRaw);
      _shooter->SetRaw(-12_V * _codriver->GetRightTriggerAxis());
      _led->SetState(LEDState::kIdle);
    } else {
      _shooter->SetState(ShooterState::kIdle);
      _led->SetState(LEDState::kIdle);
    }
  }
}

VisionShooterSpeed::VisionShooterSpeed(Shooter* shooter, Vision* vision)
    : m_shooter{shooter},
      m_vision{vision},
      m_table{nt::NetworkTableInstance::GetDefault().GetTable("shooter/visioncontrol")} {
  Controls(m_shooter);
}

units::meter_t VisionShooterSpeed::DistanceFromTarget() {
  auto distance = m_vision->GetDistanceToTag();
  auto tag = m_vision->GetID();
  switch(tag) {
    case 1:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 2:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 3:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 4:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 5:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 6:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 7:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 8:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 9:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 10:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 11:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 12:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 13:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 14:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 15:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
    case 16:
      return wom::utils::Pythagoras(distance, magicvalue);
    break;
  }
}

units::radians_per_second_t VisionShooterSpeed::GetDesiredSpeed(units::meter_t distance) {
  return units::radians_per_second_t{0};
}

void VisionShooterSpeed::OnTick(units::second_t dt) {
  m_shooter->SetState(ShooterState::kSpinUp);
  m_shooter->SetPidGoal(GetDesiredSpeed(DistanceFromTarget()));
}
