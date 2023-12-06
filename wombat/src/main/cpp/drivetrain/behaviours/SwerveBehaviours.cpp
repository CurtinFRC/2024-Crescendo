#include <drivetrain/behaviours/SwerveBehaviours.h>

wom::drivetrain::behaviours::FieldRelativeSwerveDrive::FieldRelativeSwerveDrive(
    wom::drivetrain::Swerve *swerve, frc::XboxController &driver)
    : _swerve(swerve), _driver(driver) {}

void wom::drivetrain::behaviours::FieldRelativeSwerveDrive::OnTick(units::second_t dt) {
  _swerve->SetState(wom::drivetrain::SwerveState::kFieldRelative);
  frc::Pose3d currentPose = _swerve->GetLimelight()->GetPose();
  frc::Pose3d desiredPose =
      frc::Pose3d(currentPose.X() + units::meter_t{_driver.GetRightX()},
                  currentPose.Y() + units::meter_t{_driver.GetRightY()}, currentPose.Z(),
                  frc::Rotation3d(currentPose.Rotation().X(), currentPose.Rotation().Y(),
                                  currentPose.Rotation().Z() +
                                      units::radian_t{std::atan((_driver.GetLeftY() / _driver.GetLeftX()))}));
  _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
}
