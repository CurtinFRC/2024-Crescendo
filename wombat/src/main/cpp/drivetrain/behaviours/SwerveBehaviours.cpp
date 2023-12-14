// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/behaviours/SwerveBehaviours.h"

wom::drivetrain::behaviours::FieldRelativeSwerveDrive::FieldRelativeSwerveDrive(
    wom::drivetrain::Swerve *swerve, frc::XboxController &driver)
    : Behaviour("Field Relative Swerve"), _swerve(swerve), _driver(driver) {
      Controls(swerve);
    }

void wom::drivetrain::behaviours::FieldRelativeSwerveDrive::OnTick(units::second_t dt) {
  _swerve->SetState(wom::drivetrain::SwerveState::kFieldRelative);
  std::cout << "Right X" << _driver.GetRightX() << std::endl;
  std::cout << "Right Y" << _driver.GetRightY() << std::endl;
  std::cout << "Left X" << _driver.GetLeftX() << std::endl;
  std::cout << "Left Y" << _driver.GetLeftY() << std::endl;
  // frc::Pose3d currentPose = _swerve->GetLimelight()->GetPose();
  frc::Pose3d currentPose = frc::Pose3d{0_m, 0_m, 0_m, frc::Rotation3d{0_rad, 0_rad, 0_rad}};
  frc::Pose3d desiredPose =
      frc::Pose3d(currentPose.X() + units::meter_t{_driver.GetRightX()},
                  currentPose.Y() + units::meter_t{_driver.GetRightY()}, currentPose.Z(),
                  frc::Rotation3d(currentPose.Rotation().X(), currentPose.Rotation().Y(),
                                  currentPose.Rotation().Z() +
                                      units::radian_t{std::atan((_driver.GetLeftY() / _driver.GetLeftX()))}));
  _swerve->SetDesired(desiredPose);
}
