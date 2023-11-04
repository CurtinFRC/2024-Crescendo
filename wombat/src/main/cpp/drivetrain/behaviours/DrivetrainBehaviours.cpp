#include "drivetrain/behaviours/DrivetrainBehaviours.h"

wom::drivetrain::behaviours::TankDrive::TankDrive(wom::Drivetrain *drivebase) : _drivebase(drivebase) {
  Controls(drivebase);
}

void wom::drivetrain::behaviours::TankDrive::OnTick(units::second_t dt) {
  _drivebase->SetState(wom::DrivetrainState::kTank);
}

