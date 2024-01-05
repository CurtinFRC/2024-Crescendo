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

  _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
}

wom::drivetrain::behaviours::GoToPose::GoToPose(wom::drivetrain::Swerve *swerve, frc::Pose3d pose)
    : _swerve(swerve), _pose(pose) {}

void wom::drivetrain::behaviours::GoToPose::OnTick(units::second_t dt) {
  _swerve->SetState(wom::drivetrain::SwerveState::kPose);
  frc::Pose3d currentPose = _swerve->GetLimelight()->GetPose();
  frc::Pose3d desiredPose = _pose;

  _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
}

wom::drivetrain::behaviours::FollowTrajectory::FollowTrajectory(wom::drivetrain::Swerve *swerve, wom::utils::Pathplanner *pathplanner, std::string path)
    : _swerve(swerve), _pathplanner(pathplanner), _path(path) {}

void wom::drivetrain::behaviours::FollowTrajectory::OnTick(units::second_t dt) {
  _swerve->SetState(wom::drivetrain::SwerveState::kTrajectory);
  frc::Pose3d currentPose = _swerve->GetLimelight()->GetPose();
  frc::Pose3d desiredPose = frc::Pose3d(_trajectory.Sample(m_timer.Get()).pose);

  _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
}

void wom::drivetrain::behaviours::FollowTrajectory::OnStart() {
  _trajectory = _pathplanner->getTrajectory(_path);

  m_timer.Reset();
  m_timer.Start();
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

frc::Pose3d wom::drivetrain::behaviours::TempSimSwerveDrive::GetPose() {
  frc::Pose3d currentPose{m_driveSim.GetPose()};
  return currentPose;
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

wom::drivetrain::behaviours::AutoSwerveDrive::AutoSwerveDrive(wom::drivetrain::Swerve *swerve, frc::Timer *timer, frc::Field2d *field) : _swerve(swerve), m_timer(timer), m_field(field) {
  _simSwerveDrive = new wom::drivetrain::behaviours::TempSimSwerveDrive(timer, field);
}

void wom::drivetrain::behaviours::AutoSwerveDrive::OnUpdate() {
  _simSwerveDrive->OnUpdate();
  _swerve->SetState(wom::drivetrain::SwerveState::kFieldRelative);
  //_swerve->SetDesired(_simSwerveDrive->GetPose());
}

void wom::drivetrain::behaviours::AutoSwerveDrive::SetPath(std::string path) {
  _simSwerveDrive->SetPath(path);
}