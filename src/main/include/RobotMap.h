#pragma once

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

#include "Wombat.h"

struct RobotMap {
  struct Controllers {
    frc::XboxController driver         = frc::XboxController(0);
    frc::XboxController coDriver       = frc::XboxController(1);
    frc::XboxController testController = frc::XboxController(2);
  };
  Controllers controllers;

  WPI_TalonFX            frontLeftMovementMotor{99};
  MotorVoltageController frontLeftMovementVoltageController{&frontLeftMovementMotor};
  TalonFXEncoder         frontLeftMovementEncoder{&frontLeftMovementMotor, 100};
  Gearbox                frontLeftMovement{&frontLeftMovementVoltageController, &frontLeftMovementEncoder,
                            frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX            frontLeftRotationMotor{99};
  MotorVoltageController frontLeftRotationVoltageController{&frontLeftMovementMotor};
  TalonFXEncoder         frontLeftRotationEncoder{&frontLeftMovementMotor, 100};
  Gearbox                frontLeftRotation{&frontLeftMovementVoltageController, &frontLeftMovementEncoder,
                            frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  SwerveModuleConfig frontLeftConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                     frontLeftMovement,
                                     frontLeftRotation,
                                     PIDConfig<units::radians_per_second, units::volt>(""),
                                     PIDConfig<units::meters_per_second, units::volt>(""),
                                     PIDConfig<units::radian, units::radians_per_second>(""),
                                     PIDConfig<units::meter, units::meters_per_second>(""),
                                     units::meter_t{0.05},
                                     SwerveModuleName::FrontLeft,
                                     ""};

  SwerveModule frontLeft = SwerveModule(frontLeftConfig, SwerveModuleState::kIdle);

  WPI_TalonFX            frontRightMovementMotor{99};
  MotorVoltageController frontRightMovementVoltageController{&frontRightMovementMotor};
  TalonFXEncoder         frontRightMovementEncoder{&frontRightMovementMotor, 100};
  Gearbox                frontRightMovement{&frontRightMovementVoltageController, &frontRightMovementEncoder,
                             frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX            frontRightRotationMotor{99};
  MotorVoltageController frontRightRotationVoltageController{&frontRightMovementMotor};
  TalonFXEncoder         frontRightRotationEncoder{&frontRightMovementMotor, 100};
  Gearbox                frontRightRotation{&frontRightMovementVoltageController, &frontRightMovementEncoder,
                             frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  SwerveModuleConfig frontRightConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                      frontLeftMovement,
                                      frontLeftRotation,
                                      PIDConfig<units::radians_per_second, units::volt>(""),
                                      PIDConfig<units::meters_per_second, units::volt>(""),
                                      PIDConfig<units::radian, units::radians_per_second>(""),
                                      PIDConfig<units::meter, units::meters_per_second>(""),
                                      units::meter_t{0.05},
                                      SwerveModuleName::FrontRight,
                                      ""};

  SwerveModule frontRight = SwerveModule(frontRightConfig, SwerveModuleState::kIdle);

  WPI_TalonFX            backLeftMovementMotor{99};
  MotorVoltageController backLeftMovementVoltageController{&backLeftMovementMotor};
  TalonFXEncoder         backLeftMovementEncoder{&backLeftMovementMotor, 100};
  Gearbox                backLeftMovement{&backLeftMovementVoltageController, &backLeftMovementEncoder,
                           frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX            backLeftRotationMotor{99};
  MotorVoltageController backLeftRotationVoltageController{&backLeftMovementMotor};
  TalonFXEncoder         backLeftRotationEncoder{&backLeftMovementMotor, 100};
  Gearbox                backLeftRotation{&backLeftMovementVoltageController, &backLeftMovementEncoder,
                           frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  SwerveModuleConfig backLeftConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                    frontLeftMovement,
                                    frontLeftRotation,
                                    PIDConfig<units::radians_per_second, units::volt>(""),
                                    PIDConfig<units::meters_per_second, units::volt>(""),
                                    PIDConfig<units::radian, units::radians_per_second>(""),
                                    PIDConfig<units::meter, units::meters_per_second>(""),
                                    units::meter_t{0.05},
                                    SwerveModuleName::BackLeft,
                                    ""};

  SwerveModule backLeft = SwerveModule(backLeftConfig, SwerveModuleState::kIdle);

  WPI_TalonFX            backRightMovementMotor{99};
  MotorVoltageController backRightMovementVoltageController{&backRightMovementMotor};
  TalonFXEncoder         backRightMovementEncoder{&backRightMovementMotor, 100};
  Gearbox                backRightMovement{&backRightMovementVoltageController, &backRightMovementEncoder,
                            frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX            backRightRotationMotor{99};
  MotorVoltageController backRightRotationVoltageController{&backRightMovementMotor};
  TalonFXEncoder         backRightRotationEncoder{&backRightMovementMotor, 100};
  Gearbox                backRightRotation{&backRightMovementVoltageController, &backRightMovementEncoder,
                            frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  SwerveModuleConfig backRightConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                     frontLeftMovement,
                                     frontLeftRotation,
                                     PIDConfig<units::radians_per_second, units::volt>(""),
                                     PIDConfig<units::meters_per_second, units::volt>(""),
                                     PIDConfig<units::radian, units::radians_per_second>(""),
                                     PIDConfig<units::meter, units::meters_per_second>(""),
                                     units::meter_t{0.05},
                                     SwerveModuleName::BackRight,
                                     ""};

  SwerveModule backRight = SwerveModule(backRightConfig, SwerveModuleState::kIdle);

  SwerveConfig swerveConfig{frontLeft, frontRight, backLeft, backRight};

  Limelight limelight = Limelight("Limelight");
  Swerve    swerve    = Swerve(swerveConfig, SwerveState::kIdle, &limelight);
};
