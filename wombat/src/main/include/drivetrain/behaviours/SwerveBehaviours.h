// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/Encoder.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <memory>
#include <string>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "utils/Pathplanner.h"

namespace wom {
namespace drivetrain {
namespace behaviours {

/**
 * @brief Behaviour class to handle manual drivebase controlling with the
 * controller
 */
class ManualDrivebase : public behaviour::Behaviour {
 public:
  /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase (the allocated memory address that stores
   * the "swerve drivebase" object)
   * @param driverController
   * A pointer to the controller that the driver has been allocated (the
   * allocated memory address that stores the "driver controller" object)
   */
  ManualDrivebase(wom::drivetrain::SwerveDrive* swerveDrivebase,
                  frc::XboxController* driverController);

  void OnTick(units::second_t deltaTime) override;
  /**
   * @brief This function handles all of the logic behind the tangent function,
   * to be able to calculate an angle between 0 andd 360 degrees, inclusively
   */
  void CalculateRequestedAngle(double joystickX, double joystickY,
                               units::degree_t defaultAngle);
  void OnStart() override;
  void ResetMode();

 private:
  std::shared_ptr<nt::NetworkTable> _swerveDriveTable =
      nt::NetworkTableInstance::GetDefault().GetTable("swerve");
  wom::drivetrain::SwerveDrive* _swerveDrivebase;
  frc::XboxController* _driverController;

  // State-handler Boolean : Is the robot in field orientated control, or robot
  // relative?
  bool isFieldOrientated = true;
  // State-handler Boolean : Do we currently want the angles of the wheels to be
  // 0?
  bool isZero = false;
  bool resetMode = false;

  units::degree_t _requestedAngle;
  bool isRotateMatch = false;

  // Deadzones
  const double driverDeadzone = 0.08;
  const double turningDeadzone = 0.2;

  // Variables for solution to Anti-tip
  double prevJoystickX, prevJoystickY, prevPrevJoystickX, prevPrevJoystickY,
      usingJoystickXPos, usingJoystickYPos;
  // The speed that the joystick must travel to activate averaging over previous
  // 3 joystick positions
  const double smoothingThreshold = 1;

  typedef units::meters_per_second_t translationSpeed_;
  typedef units::radians_per_second_t rotationSpeed_;

  // The translation speeds for when "slow speed", "normal speed", "fast speed"
  // modes are active
  const translationSpeed_ lowSensitivityDriveSpeed = 3.25_ft / 1_s;
  const translationSpeed_ defaultDriveSpeed = 13_ft / 1_s;
  const translationSpeed_ highSensitivityDriveSpeed = 18_ft / 1_s;
  // The rotation speeds for when "slow speed", "normal speed", "fast speed"
  // modes are active
  const rotationSpeed_ lowSensitivityRotateSpeed = 90_deg / 1_s;
  const rotationSpeed_ defaultRotateSpeed = 360_deg / 1_s;
  const rotationSpeed_ highSensitivityRotateSpeed = 720_deg / 1_s;

  translationSpeed_ maxMovementMagnitude = defaultDriveSpeed;
  rotationSpeed_ maxRotationMagnitude = defaultRotateSpeed;
};

/**
 * @brief Behaviour Class to handle locking wheels
 */
class XDrivebase : public behaviour::Behaviour {
 public:
  /**
   * @param swerveDrivebase
   * A pointer to the swerve drivebase
   */
  explicit XDrivebase(wom::drivetrain::SwerveDrive* swerveDrivebase);

  void OnTick(units::second_t deltaTime) override;

 private:
  wom::drivetrain::SwerveDrive* _swerveDrivebase;
};

class GoToPose : public behaviour::Behaviour {
 public:
  GoToPose(wom::drivetrain::SwerveDrive* swerve, frc::Pose3d pose);

  void OnTick(units::second_t dt) override;

 private:
  wom::drivetrain::SwerveDrive* _swerve;
  frc::Pose3d _pose;
};

class FollowTrajectory : public behaviour::Behaviour {
 public:
  FollowTrajectory(wom::drivetrain::SwerveDrive* swerve,
                   wom::utils::Pathplanner* pathplanner, std::string path);

  void OnTick(units::second_t dt) override;

  void OnStart() override;

 private:
  wom::utils::Pathplanner* _pathplanner;
  std::string _path;
  wom::drivetrain::SwerveDrive* _swerve;
  frc::Trajectory _trajectory;
  frc::Timer m_timer;
};

class TempSimSwerveDrive {
 public:
  TempSimSwerveDrive(frc::Timer* timer, frc::Field2d* field);

  void OnUpdate();

  void SetPath(std::string path);

  frc::Pose3d GetPose();
  frc::Pose2d GetPose2d();

 private:
  frc::sim::DifferentialDrivetrainSim m_driveSim{
      frc::DCMotor::NEO(2),  // 2 NEO motors on each side of the drivetrain.
      7.29,                  // 7.29:1 gearing reduction.
      7.5_kg_sq_m,           // MOI of 7.5 kg m^2 (from CAD model).
      60_kg,                 // The mass of the robot is 60 kg.
      3_in,                  // The robot uses 3" radius wheels.
      0.7112_m,              // The track width is 0.7112 meters.

      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};

  wom::utils::Pathplanner m_pathplanner;

  frc::Trajectory current_trajectory;

  std::shared_ptr<nt::NetworkTable> current_trajectory_table;
  std::shared_ptr<nt::NetworkTable> current_trajectory_state_table;

  frc::Timer* m_timer;

  frc::Field2d* m_field;

  std::string m_path;
};

class AutoSwerveDrive {
 public:
  AutoSwerveDrive(wom::drivetrain::SwerveDrive* swerve, frc::Timer* timer,
                  frc::Field2d* field);

  void OnUpdate();

  void SetPath(std::string path);

 private:
  wom::drivetrain::SwerveDrive* _swerve;

  TempSimSwerveDrive* _simSwerveDrive;

  frc::Timer* m_timer;

  frc::Field2d* m_field;

  std::string m_path;
};
}  // namespace behaviours
}  // namespace drivetrain
}  // namespace wom
