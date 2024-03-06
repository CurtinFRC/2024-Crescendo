// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/math.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/SymbolExports.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <memory>
#include <string>
#include <vector>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

#include "behaviour/HasBehaviour.h"
#include "units/angle.h"
#include "utils/Gearbox.h"
#include "utils/PID.h"

namespace wom {
namespace drivetrain {

/**
 * Implements a PID control loop.
 */
class PIDController : public wpi::Sendable, public wpi::SendableHelper<PIDController> {
 public:
  /**
   * Allocates a PIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param Kp     The proportional coefficient. Must be >= 0.
   * @param Ki     The integral coefficient. Must be >= 0.
   * @param Kd     The derivative coefficient. Must be >= 0.
   * @param period The period between controller updates in seconds. The
   *               default is 20 milliseconds. Must be positive.
   */
  PIDController(double Kp, double Ki, double Kd, units::second_t period = 20_ms);

  ~PIDController() override = default;

  PIDController(const PIDController&) = default;
  PIDController& operator=(const PIDController&) = default;
  PIDController(PIDController&&) = default;
  PIDController& operator=(PIDController&&) = default;

  /**
   * Sets the PID Controller gain parameters.
   *
   * Sets the proportional, integral, and differential coefficients.
   *
   * @param Kp The proportional coefficient. Must be >= 0.
   * @param Ki The integral coefficient. Must be >= 0.
   * @param Kd The differential coefficient. Must be >= 0.
   */
  void SetPID(double Kp, double Ki, double Kd);

  /**
   * Sets the proportional coefficient of the PID controller gain.
   *
   * @param Kp The proportional coefficient. Must be >= 0.
   */
  void SetP(double Kp);

  /**
   * Sets the integral coefficient of the PID controller gain.
   *
   * @param Ki The integral coefficient. Must be >= 0.
   */
  void SetI(double Ki);

  /**
   * Sets the differential coefficient of the PID controller gain.
   *
   * @param Kd The differential coefficient. Must be >= 0.
   */
  void SetD(double Kd);

  /**
   * Sets the IZone range. When the absolute value of the position error is
   * greater than IZone, the total accumulated error will reset to zero,
   * disabling integral gain until the absolute value of the position error is
   * less than IZone. This is used to prevent integral windup. Must be
   * non-negative. Passing a value of zero will effectively disable integral
   * gain. Passing a value of infinity disables IZone functionality.
   *
   * @param iZone Maximum magnitude of error to allow integral control. Must be
   *   >= 0.
   */
  void SetIZone(double iZone);

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  double GetP() const;

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  double GetI() const;

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  double GetD() const;

  /**
   * Get the IZone range.
   *
   * @return Maximum magnitude of error to allow integral control.
   */
  double GetIZone() const;

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  units::second_t GetPeriod() const;

  /**
   * Gets the position tolerance of this controller.
   *
   * @return The position tolerance of the controller.
   */
  double GetPositionTolerance() const;

  /**
   * Gets the velocity tolerance of this controller.
   *
   * @return The velocity tolerance of the controller.
   */
  double GetVelocityTolerance() const;

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  void SetSetpoint(double setpoint);

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  double GetSetpoint() const;

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * This will return false until at least one input value has been computed.
   */
  bool AtSetpoint() const;

  /**
   * Enables continuous input.
   *
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route
   * to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  void EnableContinuousInput(double minimumInput, double maximumInput);

  /**
   * Disables continuous input.
   */
  void DisableContinuousInput();

  /**
   * Returns true if continuous input is enabled.
   */
  bool IsContinuousInputEnabled() const;

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * When the cap is reached, the integrator value is added to the controller
   * output rather than the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  void SetIntegratorRange(double minimumIntegral, double maximumIntegral);

  /**
   * Sets the error which is considered tolerable for use with AtSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  void SetTolerance(double positionTolerance,
                    double velocityTolerance = std::numeric_limits<double>::infinity());

  /**
   * Returns the difference between the setpoint and the measurement.
   */
  double GetPositionError() const;

  /**
   * Returns the velocity error.
   */
  double GetVelocityError() const;

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  double Calculate(double measurement);

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param setpoint The new setpoint of the controller.
   */
  double Calculate(double measurement, double setpoint);

  /**
   * Reset the previous error, the integral term, and disable the controller.
   */
  void Reset();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  // Factor for "proportional" control
  double m_Kp;

  // Factor for "integral" control
  double m_Ki;

  // Factor for "derivative" control
  double m_Kd;

  // The error range where "integral" control applies
  double m_iZone = std::numeric_limits<double>::infinity();

  // The period (in seconds) of the control loop running this controller
  units::second_t m_period;

  double m_maximumIntegral = 1.0;

  double m_minimumIntegral = -1.0;

  double m_maximumInput = 0;

  double m_minimumInput = 0;

  // Do the endpoints wrap around? eg. Absolute encoder
  bool m_continuous = false;

  // The error at the time of the most recent call to Calculate()
  double m_positionError = 0;
  double m_velocityError = 0;

  // The error at the time of the second-most-recent call to Calculate() (used
  // to compute velocity)
  double m_prevError = 0;

  // The sum of the errors for use in the integral calc
  double m_totalError = 0;

  // The error that is considered at setpoint.
  double m_positionTolerance = 0.2;
  double m_velocityTolerance = std::numeric_limits<double>::infinity();

  double m_setpoint = 0;
  double m_measurement = 0;

  bool m_haveSetpoint = false;
  bool m_haveMeasurement = false;
};

enum class SwerveModuleState { kZeroing, kIdle, kPID };
enum class TurnOffsetValues { reverse, forward, none };

struct SwerveModuleConfig {
  frc::Translation2d position;

  utils::Gearbox driveMotor;
  utils::Gearbox turnMotor;

  ctre::phoenix6::hardware::CANcoder* canEncoder;

  units::meter_t wheelRadius;

  void WriteNT(std::shared_ptr<nt::NetworkTable> table) const;
};

class SwerveModule {
 public:
  // using angle_pid_conf_t = utils::PIDConfig<units::radian, units::volt>;
  using velocity_pid_conf_t = utils::PIDConfig<units::meters_per_second, units::volt>;

  SwerveModule(std::string path, SwerveModuleConfig config,
               /*angle_pid_conf_t anglePID,*/ velocity_pid_conf_t velocityPID);
  void OnUpdate(units::second_t dt);
  void OnStart();

  /**
   * @brief This function acts to aid the robot matching the joystick's angle
   * @param speeds
   * Contains the xVelocity, yVelocity and the rotationalSpeed that the robot
   * will be moving in
   */
  void ModuleVectorHandler(frc::ChassisSpeeds speeds);

  void SetZero(units::second_t dt);
  void SetIdle();
  void SetPID(units::radian_t angle, units::meters_per_second_t speed, units::second_t dt);
  void SetZero();
  void SetVoltageLimit(units::volt_t driveModuleVoltageLimit);

  void SetTurnOffsetForward();
  void SetTurnOffsetReverse();
  void TurnOffset();

  // double GetCancoderPosition(); // from liam's

  void SetAccelerationLimit(units::meters_per_second_squared_t limit);

  // frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition() const;

  units::meters_per_second_t GetSpeed() const;
  units::meter_t GetDistance() const;

  const SwerveModuleConfig& GetConfig() const;

  // utils::PIDController<units::radians, units::volt> _anglePIDController;
  frc::PIDController _anglePIDController;

 private:
  SwerveModuleConfig _config;
  SwerveModuleState _state;
  units::volt_t _driveModuleVoltageLimit = 10_V;
  units::volt_t _angleModuleVoltageLimit = 6_V;

  bool _hasZeroedEncoder = false;
  bool _hasZeroed = false;

  frc::PIDController _velocityPIDController;

  std::shared_ptr<nt::NetworkTable> _table;

  double startingPos;

  double _offset;
  units::meters_per_second_squared_t _currentAccelerationLimit = 15_mps / 1_s;

  TurnOffsetValues _turnOffset = TurnOffsetValues::none;
};

struct SwerveDriveConfig {
  /*using pose_angle_conf_t =
      utils::PIDConfig<units::radian, units::radians_per_second>;*/
  using pose_position_conf_t = utils::PIDConfig<units::meter, units::meters_per_second>;
  using balance_conf_t = utils::PIDConfig<units::degree, units::meters_per_second>;

  std::string path;
  // SwerveModule::angle_pid_conf_t anglePID;
  SwerveModule::velocity_pid_conf_t velocityPID;

  wpi::array<SwerveModuleConfig, 4> modules;

  ctre::phoenix6::hardware::Pigeon2* gyro;

  // pose_angle_conf_t poseAnglePID;
  pose_position_conf_t posePositionPID;

  units::kilogram_t mass;

  wpi::array<double, 3> stateStdDevs{0.0, 0.0, 0.0};
  wpi::array<double, 3> visionMeasurementStdDevs{0.0, 0.0, 0.0};

  void WriteNT(std::shared_ptr<nt::NetworkTable> table);
};

enum class SwerveDriveState {
  kZeroing,
  kIdle,
  kAngle,
  kVelocity,
  kFieldRelativeVelocity,
  kPose,
  kIndividualTuning,
  kTuning,
  kXWheels,
  kModuleTurn,
  kFRVelocityRotationLock
};

struct FieldRelativeSpeeds {
  /**
   * Represents the velocity in the x dimension (your alliance to opposite
   * alliance)
   */
  units::meters_per_second_t vx{0};
  /**
   * Represents the velocity in the y dimension (to your left when standing
   * behind alliance wall)
   */
  units::meters_per_second_t vy{0};
  /**
   * The angular velocity of the robot (CCW is +)
   */
  units::radians_per_second_t omega{0};

  frc::ChassisSpeeds ToChassisSpeeds(const units::radian_t robotHeading);
};

class SwerveDrive : public behaviour::HasBehaviour {
 public:
  SwerveDrive(SwerveDriveConfig config, frc::Pose2d initialPose);

  void OnUpdate(units::second_t dt);
  void OnStart();

  /**
   * @brief This function switches the state to handle the robot's rotation
   * matching that of the joystick
   */
  void RotateMatchJoystick(units::radian_t joystickAngle, FieldRelativeSpeeds speeds);

  void SetIdle();

  void SetZeroing();

  void SetVelocity(frc::ChassisSpeeds speeds);
  void SetFieldRelativeVelocity(FieldRelativeSpeeds speeds);
  void SetPose(frc::Pose2d pose);
  bool IsAtSetPose();
  void SetIndividualTuning(int mod, units::radian_t angle, units::meters_per_second_t speed);
  void SetTuning(units::radian_t angle, units::meters_per_second_t speed);
  void SetZero();
  void SetVoltageLimit(units::volt_t driveVoltageLimit);
  void OnResetMode();

  SwerveDriveState GetState();
  bool IsAtSetAngle();

  // double GetModuleCANPosition(int mod);  // from liam's

  void SetXWheelState();

  void SetIsFieldRelative(bool value);
  bool GetIsFieldRelative();

  void SetAccelerationLimit(units::meters_per_second_squared_t limit);

  void ResetPose(frc::Pose2d pose);

  frc::Pose2d GetPose();
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp);

  void TurnToAngle(units::radian_t angle);

  SwerveDriveConfig& GetConfig() { return _config; }

  frc::Pose2d GetSetpoint();

  void MakeAtSetPoint();

 private:
  SwerveDriveConfig _config;
  SwerveDriveState _state = SwerveDriveState::kIdle;
  std::vector<SwerveModule> _modules;

  units::degree_t _requestedAngle;
  FieldRelativeSpeeds _requestedSpeeds;

  double diff = 0;
  double diff2 = 0;

  double lastdiff = 0;
  double lastdiff2 = 0;

  frc::ChassisSpeeds last_new_speed;
  frc::ChassisSpeeds last_speed;

  frc::ChassisSpeeds _target_speed;
  FieldRelativeSpeeds _target_fr_speeds;

  frc::SwerveDriveKinematics<4> _kinematics;
  frc::SwerveDrivePoseEstimator<4> _poseEstimator;

  /*utils::PIDController<units::radian, units::radians_per_second>
      _anglePIDController;*/
  wom::drivetrain::PIDController _anglePIDController;
  wom::drivetrain::PIDController _xPIDController;
  wom::drivetrain::PIDController _yPIDController;
  wom::drivetrain::PIDController _turnPIDController;
  // wom::utils::PIDController<units::meter, units::meters_per_second> _xPIDController;
  // wom::utils::PIDController<units::meter, units::meters_per_second> _yPIDController;

  std::shared_ptr<nt::NetworkTable> _table;

  bool _isFieldRelative = true;
  bool isRotateToMatchJoystick = false;

  int _mod;
  units::radian_t _angle;
  units::meters_per_second_t _speed;
  frc::SwerveModuleState laststates[4];
  frc::SwerveModuleState lastnewstates[4];

  units::radian_t _reqAngle;
  // double frontLeftEncoderOffset = -143.26171875;
  // double frontRightEncoderOffset = 167.87109375;
  // double backLeftEncoderOffset = -316.669921875;
  // double backRightEncoderOffset = -119.619140625;
};
}  // namespace drivetrain
}  // namespace wom
