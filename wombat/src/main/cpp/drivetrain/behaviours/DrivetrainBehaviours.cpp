// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/behaviours/DrivetrainBehaviours.h"

wom::drivetrain::behaviours::TankDrive::TankDrive(wom::drivetrain::Drivetrain *drivebase,
                                                  frc::XboxController         &driver)
    : _drivebase(drivebase), _driver(driver) {
  Controls(drivebase);
}

void wom::drivetrain::behaviours::TankDrive::OnTick(units::second_t dt) {
  wom::drivetrain::TankSpeed speed;
  speed.right = wom::utils::deadzone(_driver.GetRightY());
  speed.left  = wom::utils::deadzone(_driver.GetLeftY());
  _drivebase->SetSpeed(speed);
  _drivebase->SetState(wom::drivetrain::DrivetrainState::kTank);
}
