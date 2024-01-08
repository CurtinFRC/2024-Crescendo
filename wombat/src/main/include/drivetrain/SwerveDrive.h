// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <ctre/Phoenix.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

#include <iostream>
#include <memory>
#include <string>

#include "behaviour/HasBehaviour.h"
#include "utils/Gearbox.h"
#include "utils/PID.h"
#include "utils/Util.h"
#include "utils/Gyro.h"
#include "vision/Limelight.h"

#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include "utils/VoltageController.h"
#include <frc/interfaces/Gyro.h>
#include "utils/PID.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>


namespace wom {
namespace drivetrain {
  
  enum class SwerveModuleState {
    kZeroing,
    kIdle, 
    kPID
  };

  struct SwerveModuleConfig {
    frc::Translation2d position;

    utils::Gearbox driveMotor;
    utils::Gearbox turnMotor;

    CANCoder *canEncoder;

    units::meter_t wheelRadius;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table) const;
  };

  class SwerveModule {
   public:
    using angle_pid_conf_t = utils::PIDConfig<units::radian, units::volt>;
    using velocity_pid_conf_t = utils::PIDConfig<units::meters_per_second, units::volt>;

    SwerveModule(std::string path, SwerveModuleConfig config, angle_pid_conf_t anglePID, velocity_pid_conf_t velocityPID);
    void OnUpdate(units::second_t dt);
    void OnStart();

    /**
     * @brief This function acts to aid the robot matching the joystick's angle
     * @param speeds
     * Contains the xVelocity, yVelocity and the rotationalSpeed that the robot will be moving in
    */
    void ModuleVectorHandler(frc::ChassisSpeeds speeds);

    void SetZero(units::second_t dt);
    void SetIdle();
    void SetPID(units::radian_t angle, units::meters_per_second_t speed, units::second_t dt);
    void SetZero();  
    void SetVoltageLimit(units::volt_t driveModuleVoltageLimit);

    //double GetCancoderPosition(); // from liam's


    void SetAccelerationLimit(units::meters_per_second_squared_t limit);

    // frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition() const;

    units::meters_per_second_t GetSpeed() const;
    units::meter_t GetDistance() const;

    const SwerveModuleConfig &GetConfig() const;

    utils::PIDController<units::radians, units::volt> _anglePIDController;
   private:
    SwerveModuleConfig _config;
    SwerveModuleState _state;
    units::volt_t _driveModuleVoltageLimit = 10_V;

    bool _hasZeroedEncoder = false; 
    bool _hasZeroed = false;

    utils::PIDController<units::meters_per_second, units::volt> _velocityPIDController;

    std::shared_ptr<nt::NetworkTable> _table;

    double startingPos;

    double _offset;
    units::meters_per_second_squared_t _currentAccelerationLimit = 6_mps / 1_s;
  };

  struct SwerveDriveConfig {
    using pose_angle_conf_t = utils::PIDConfig<units::radian, units::radians_per_second>;
    using pose_position_conf_t = utils::PIDConfig<units::meter, units::meters_per_second>;
    using balance_conf_t = utils::PIDConfig<units::degree, units::meters_per_second>;

    std::string path;
    SwerveModule::angle_pid_conf_t anglePID;
    SwerveModule::velocity_pid_conf_t velocityPID;

    wpi::array<SwerveModuleConfig, 4> modules;

    wom::utils::Gyro *gyro;


    pose_angle_conf_t poseAnglePID;
    pose_position_conf_t posePositionPID;

    units::kilogram_t mass;

    wpi::array<double, 3> stateStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 3> visionMeasurementStdDevs{0.0, 0.0, 0.0};

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  enum class SwerveDriveState {
    kZeroing,
    kIdle, 
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
     * Represents the velocity in the x dimension (your alliance to opposite alliance)
     */
    units::meters_per_second_t vx{0};
    /**
     * Represents the velocity in the y dimension (to your left when standing behind alliance wall)
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
     * @brief This function switches the state to handle the robot's rotation matching that of the joystick
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
    // double GetModuleCANPosition(int mod);  // from liam's

    void SetXWheelState();

    void SetIsFieldRelative(bool value);
    bool GetIsFieldRelative();

    void SetAccelerationLimit(units::meters_per_second_squared_t limit);

    void ResetPose(frc::Pose2d pose);

    frc::Pose2d GetPose();
    void AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp);

    SwerveDriveConfig &GetConfig() { return _config; }

   protected:

   private:
    SwerveDriveConfig _config;
    SwerveDriveState _state = SwerveDriveState::kIdle;
    std::vector<SwerveModule> _modules;

    units::degree_t _requestedAngle;
    FieldRelativeSpeeds _requestedSpeeds;

    frc::ChassisSpeeds _target_speed;
    FieldRelativeSpeeds _target_fr_speeds;

    frc::SwerveDriveKinematics<4> _kinematics;
    frc::SwerveDrivePoseEstimator<4> _poseEstimator;

    utils::PIDController<units::radian, units::radians_per_second> _anglePIDController;
    utils::PIDController<units::meter, units::meters_per_second> _xPIDController;
    utils::PIDController<units::meter, units::meters_per_second> _yPIDController;

    std::shared_ptr<nt::NetworkTable> _table;

    bool _isFieldRelative = true;
    bool isRotateToMatchJoystick = false;

    int _mod;
    units::radian_t _angle;
    units::meters_per_second_t _speed;

    double frontLeftEncoderOffset = -143.26171875;
    double frontRightEncoderOffset = 167.87109375;
    double backLeftEncoderOffset = -316.669921875;
    double backRightEncoderOffset = -119.619140625;
  };
}  // namespace drivetrain
}  // namespace wom
