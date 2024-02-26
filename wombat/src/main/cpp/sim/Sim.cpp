#include "sim/Sim.h"
#include "drivetrain/SwerveDrive.h"
#include "frc/smartdashboard/SmartDashboard.h"

wom::sim::SimSwerve::SimSwerve(wom::drivetrain::SwerveDrive* _swerve) : _swerve(_swerve) {
  frc::SmartDashboard::PutData("Field", &_field);
}

void wom::sim::SimSwerve::OnTick() {
  _field.SetRobotPose(_swerve->GetPose());
}
