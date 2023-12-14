// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/behaviours/SwerveBehaviours.h"
#include "utils/Pathplanner.h"
#include "utils/Util.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/Timer.h>

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

  // _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
}



wom::drivetrain::behaviours::TempSimSwerveDrive::TempSimSwerveDrive(frc::Timer *timer, frc::Field2d *field)
    : m_timer(timer), m_field(field) {}

void wom::drivetrain::behaviours::TempSimSwerveDrive::OnUpdate() {
  m_field->SetRobotPose(m_driveSim.GetPose());

  // get the current trajectory state
  frc::Trajectory::State desired_state = current_trajectory.Sample(m_timer->Get());

  // get the current pose of the robot
  frc::Pose2d current_pose = m_driveSim.GetPose();
    
  // get the current wheel speeds
  wom::utils::WriteTrajectoryState(current_trajectory_state_table, desired_state);

  // move drivebase position to the desired state
  m_driveSim.SetPose(wom::utils::TrajectoryStateToPose2d(desired_state));
    
  // update the drivebase
  m_driveSim.Update(20_ms);
}

void wom::drivetrain::behaviours::TempSimSwerveDrive::SetPath(std::string path) {
  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("FMSInfo");

  // create a netowrk table for the trajectory
  std::shared_ptr<nt::NetworkTable> trajectory_table = nt::NetworkTableInstance::GetDefault().GetTable("trajectory_path");
  current_trajectory_table = nt::NetworkTableInstance::GetDefault().GetTable("current_trajectory");
  current_trajectory_state_table = nt::NetworkTableInstance::GetDefault().GetTable("current_trajectory_state");

  current_trajectory = m_pathplanner.getTrajectory(path);
  m_driveSim.SetPose(current_trajectory.Sample(0_s).pose);
  m_timer->Reset();
  m_timer->Start();
}