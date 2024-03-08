// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/behaviours/SwerveBehaviours.h"

#include <frc/PS4Controller.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

#include "drivetrain/SwerveDrive.h"
#include "networktables/NetworkTableInstance.h"
#include "units/angle.h"
#include "utils/Pathplanner.h"
#include "utils/Util.h"

namespace wom {
namespace drivetrain {
namespace behaviours {

// Code for Manual Drivebase
ManualDrivebase::ManualDrivebase(wom::drivetrain::SwerveDrive* swerveDrivebase,
                                 frc::XboxController* driverController)
    : _swerveDrivebase(swerveDrivebase), _driverController(driverController) {
  Controls(swerveDrivebase);
}

void ManualDrivebase::OnStart() {
  _swerveDrivebase->OnStart();
  _swerveDrivebase->SetAccelerationLimit(10_mps_sq);
}

void ManualDrivebase::OnTick(units::second_t deltaTime) {
  // if (_driverController->GetXButtonPressed()) {
  //   ResetMode();
  //   isRotateMatch = !isRotateMatch;
  // }

  if (_driverController->GetYButton()) {
    _swerveDrivebase->ResetPose(frc::Pose2d());
  }

  // if (_driverController->GetLeftBumperPressed()) {
  //   maxMovementMagnitude = lowSensitivityDriveSpeed;
  //   maxRotationMagnitude = lowSensitivityRotateSpeed;
  // } else if (_driverController->GetLeftBumperReleased() &&
  //            !_driverController->GetRightBumper()) {
  //   maxMovementMagnitude = defaultDriveSpeed;
  //   maxRotationMagnitude = defaultRotateSpeed;
  //   _swerveDrivebase->SetAccelerationLimit(6_mps_sq);
  //   _swerveDrivebase->SetVoltageLimit(10_V);
  // }
  // if (_driverController->GetRightBumperPressed()) {
  //   maxMovementMagnitude = highSensitivityDriveSpeed;
  //   maxRotationMagnitude = highSensitivityRotateSpeed;
  //   _swerveDrivebase->SetAccelerationLimit(12_mps_sq);
  //   _swerveDrivebase->SetVoltageLimit(14_V);

  // } else if (_driverController->GetRightBumperReleased() &&
  //            !_driverController->GetLeftBumper()) {
  //   maxMovementMagnitude = defaultDriveSpeed;
  //   maxRotationMagnitude = defaultRotateSpeed;
  //   _swerveDrivebase->SetAccelerationLimit(6_mps_sq);
  //   _swerveDrivebase->SetVoltageLimit(10_V);
  // }

  // if (_driverController->GetAButtonReleased()) {
  //   isZero = !isZero;
  // }

  // if (isZero) {
  //   _swerveDrivebase->SetZeroing();
  // } else {
  double xVelocity = wom::utils::spow2(
      -wom::utils::deadzone(_driverController->GetLeftY(),
                            driverDeadzone));  // GetLeftY due to x being where y should be on field
  double yVelocity = wom::utils::spow2(-wom::utils::deadzone(_driverController->GetLeftX(), driverDeadzone));

  double r_x = wom::utils::spow2(-wom::utils::deadzone(_driverController->GetRightX(), turningDeadzone));

  double turnX = _driverController->GetRightX();
  double turnY = _driverController->GetRightY();
  double num = std::sqrt(turnX * turnX + turnY * turnY);
  if (num < turningDeadzone) {
    turnX = 0;
    turnY = 0;
  }

  // if (isRotateMatch) {
  //   units::degree_t currentAngle =
  //       _swerveDrivebase->GetPose().Rotation().Degrees();
  //   CalculateRequestedAngle(turnX, turnY, currentAngle);
  //   _swerveDriveTable->GetEntry("RotateMatch")
  //       .SetDouble(_requestedAngle.value());
  //   _swerveDrivebase->RotateMatchJoystick(
  //       _requestedAngle,
  //       wom::drivetrain::FieldRelativeSpeeds{// also field relative
  //                                            xVelocity * maxMovementMagnitude,
  //                                            yVelocity * maxMovementMagnitude,
  //                                            r_x * maxRotationMagnitude});
  // } else {
  _swerveDrivebase->SetFieldRelativeVelocity(wom::drivetrain::FieldRelativeSpeeds{
      xVelocity * -maxMovementMagnitude, yVelocity * -maxMovementMagnitude, r_x * -maxRotationMagnitude});

  //  _swerveDrivebase->SetVelocity(
  //       frc::ChassisSpeeds{xVelocity * maxMovementMagnitude,
  //                          yVelocity * maxMovementMagnitude,
  //                          r_x * maxRotationMagnitude});
  //   }
  // }
  // _swerveDrivebase->SetIndividualTuning(2, 0_deg, 0_mps);
}

void ManualDrivebase::ResetMode() {
  _swerveDrivebase->OnResetMode();
  resetMode = false;
}

void ManualDrivebase::CalculateRequestedAngle(double joystickX, double joystickY,
                                              units::degree_t defaultAngle) {
  _requestedAngle = (1_rad * std::atan2(joystickY, -joystickX)) + 90_deg;
  if (wom::utils::deadzone(joystickX) == 0 && wom::utils::deadzone(joystickY) == 0) {
    _requestedAngle = _swerveDrivebase->GetPose().Rotation().Radians();
  }
}

// Code for x-ing the wheels on the drivebase
XDrivebase::XDrivebase(wom::drivetrain::SwerveDrive* swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {
  Controls(swerveDrivebase);
}
void XDrivebase::OnTick(units::second_t deltaTime) {
  _swerveDrivebase->SetXWheelState();
}
}  // namespace behaviours
}  // namespace drivetrain
}  // namespace wom
// wom::drivetrain::behaviours::GoToPose::GoToPose(wom::drivetrain::SwerveDrive
// *swerve, frc::Pose3d pose)
//     : _swerve(swerve), _pose(pose) {}

// void wom::drivetrain::behaviours::GoToPose::OnTick(units::second_t dt) {
//   _swerve->SetPose(wom::drivetrain::SwerveDriveState::kPose);
//   frc::Pose3d currentPose = _swerve->GetLimelight()->GetPose();
//   frc::Pose3d desiredPose = _pose;

//   _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
// }

// wom::drivetrain::behaviours::FollowTrajectory::FollowTrajectory(wom::drivetrain::Swerve
// *swerve, wom::utils::Pathplanner *pathplanner, std::string path)
//     : _swerve(swerve), _pathplanner(pathplanner), _path(path) {}

// void wom::drivetrain::behaviours::FollowTrajectory::OnTick(units::second_t
// dt) {
//   _swerve->SetState(wom::drivetrain::SwerveState::kTrajectory);
//   frc::Pose3d currentPose = _swerve->GetLimelight()->GetPose();
//   frc::Pose3d desiredPose =
//   frc::Pose3d(_trajectory.Sample(m_timer.Get()).pose);

//   _swerve->OnUpdate(dt, _swerve->GetLimelight(), desiredPose);
// }

// void wom::drivetrain::behaviours::FollowTrajectory::OnStart() {
//   _trajectory = _pathplanner->getTrajectory(_path);

//   m_timer.Reset();
//   m_timer.Start();
// }

// wom::drivetrain::behaviours::TempSimSwerveDrive::TempSimSwerveDrive(frc::Timer* timer, frc::Field2d* field)
//     : m_timer(timer), m_field(field) {}
//
// void wom::drivetrain::behaviours::TempSimSwerveDrive::OnUpdate() {
//   m_field->SetRobotPose(m_driveSim.GetPose());
//
//   // get the current trajectory state
//   frc::Trajectory::State desired_state = current_trajectory.Sample(m_timer->Get());
//
//   // get the current wheel speeds
//   wom::utils::WriteTrajectoryState(current_trajectory_state_table, desired_state);
//
//   // move drivebase position to the desired state
//   m_driveSim.SetPose(wom::utils::TrajectoryStateToPose2d(desired_state));
//
//   // update the drivebase
//   m_driveSim.Update(20_ms);
// }
//
// frc::Pose3d wom::drivetrain::behaviours::TempSimSwerveDrive::GetPose() {
//   frc::Pose3d currentPose{m_driveSim.GetPose()};
//   return currentPose;
// }
//
// frc::Pose2d wom::drivetrain::behaviours::TempSimSwerveDrive::GetPose2d() {
//   return m_driveSim.GetPose();
// }
//
// void wom::drivetrain::behaviours::TempSimSwerveDrive::SetPath(std::string path) {
//   nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
//   std::shared_ptr<nt::NetworkTable> table = inst.GetTable("FMSInfo");
//
//   // create a netowrk table for the trajectory
//   std::shared_ptr<nt::NetworkTable> trajectory_table =
//       nt::NetworkTableInstance::GetDefault().GetTable("trajectory_path");
//   current_trajectory_table = nt::NetworkTableInstance::GetDefault().GetTable("current_trajectory");
//   current_trajectory_state_table =
//       nt::NetworkTableInstance::GetDefault().GetTable("current_trajectory_state");
//
//   current_trajectory = m_pathplanner.getTrajectory(path);
//   m_driveSim.SetPose(current_trajectory.Sample(0_s).pose);
//   m_timer->Reset();
//   m_timer->Start();
// }
//
// wom::drivetrain::behaviours::AutoSwerveDrive::AutoSwerveDrive(wom::drivetrain::SwerveDrive* swerve,
//                                                               frc::Timer* timer, frc::Field2d* field)
//     : _swerve(swerve), m_timer(timer), m_field(field) {
//   _simSwerveDrive = new wom::drivetrain::behaviours::TempSimSwerveDrive(timer, field);
// }
//
// void wom::drivetrain::behaviours::AutoSwerveDrive::OnUpdate() {
//   _simSwerveDrive->OnUpdate();
//   _swerve->SetPose(_simSwerveDrive->GetPose2d());
// }
//
// void wom::drivetrain::behaviours::AutoSwerveDrive::SetPath(std::string path) {
//   _simSwerveDrive->SetPath(path);
// }

wom::drivetrain::behaviours::DrivebasePoseBehaviour::DrivebasePoseBehaviour(SwerveDrive* swerveDrivebase,
                                                                            frc::Pose2d pose,
                                                                            units::volt_t voltageLimit,
                                                                            bool hold)
    : _swerveDrivebase(swerveDrivebase), _pose(pose), _hold(hold), _voltageLimit(voltageLimit) {
  Controls(swerveDrivebase);
}

void wom::drivetrain::behaviours::DrivebasePoseBehaviour::OnStart() {
  if (_voltageLimit >= (frc::RobotController::GetBatteryVoltage() - 0.5_V)) {
    _voltageLimit = frc::RobotController::GetBatteryVoltage() - 1_V;
  }

  double currentAngle = _swerveDrivebase->GetPose().Rotation().Radians().value();

  units::radian_t adjustedAngle =
      1_rad * (currentAngle - std::fmod(currentAngle, 360) + _pose.Rotation().Radians().value() - 3.14159);

  _swerveDrivebase->SetVoltageLimit(_voltageLimit);

  // _swerveDrivebase->SetPose(frc::Pose2d{_pose.X(), _pose.Y(), adjustedAngle});
  _swerveDrivebase->TurnToAngle(adjustedAngle);

  _timer.Start();
}

// used in autonomous for going to set drive poses
void wom::drivetrain::behaviours::DrivebasePoseBehaviour::OnTick(units::second_t deltaTime) {
  nt::NetworkTableInstance::GetDefault().GetTable("drivetrainpose")->GetEntry("going").SetBoolean(true);
  if (_timer.Get() > 1_s) {
    nt::NetworkTableInstance::GetDefault().GetTable("drivetrainpose")->GetEntry("going").SetBoolean(false);
    SetDone();
  }
  if (_swerveDrivebase->IsAtSetAngle() && _swerveDrivebase->GetState() == SwerveDriveState::kAngle) {
    _swerveDrivebase->SetPose(frc::Pose2d(_pose.X(), _pose.Y(), 0_deg));
  } else {
    if (_swerveDrivebase->IsAtSetPose() && !_hold) {
      std::cout << "Exited..." << std::endl;
      nt::NetworkTableInstance::GetDefault().GetTable("drivetrainpose")->GetEntry("going").SetBoolean(false);
      SetDone();
    }
  }
}

wom::drivetrain::behaviours::TurnToAngleBehaviour::TurnToAngleBehaviour(wom::drivetrain::SwerveDrive* swerve, units::radian_t angle) : _angle(angle), _swerve(swerve) {
  Controls(swerve);
}

void wom::drivetrain::behaviours::TurnToAngleBehaviour::OnTick(units::second_t dt) {
  _swerve->TurnToAngle(_angle);

  nt::NetworkTableInstance::GetDefault().GetTable("drivetrain")->GetEntry("targetAngle").SetDouble(_angle.value());
  nt::NetworkTableInstance::GetDefault().GetTable("drivetrain")->GetEntry("runningangle").SetBoolean(true);

  if (units::math::abs(_swerve->GetPose().Rotation().Radians() - _angle) < 0.1_rad) {
    nt::NetworkTableInstance::GetDefault().GetTable("drivetrain")->GetEntry("runningangle").SetBoolean(false);
    SetDone();
  }
}


