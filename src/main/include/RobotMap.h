// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

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

  WPI_TalonFX                 frontLeftMovementMotor{9, "Drivebase"};
  wom::MotorVoltageController frontLeftMovementVoltageController{&frontLeftMovementMotor};
  wom::TalonFXEncoder         frontLeftMovementEncoder{&frontLeftMovementMotor};
  wom::Gearbox frontLeftMovement{&frontLeftMovementVoltageController, &frontLeftMovementEncoder,
                                 frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX                 frontLeftRotationMotor{7, "Drivebase"};
  wom::MotorVoltageController frontLeftRotationVoltageController{&frontLeftMovementMotor};
  wom::CanEncoder             frontLeftRotationEncoder{18};
  wom::Gearbox frontLeftRotation{&frontLeftMovementVoltageController, &frontLeftMovementEncoder,
                                 frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  wom::SwerveModuleConfig frontLeftConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                          frontLeftMovement,
                                          frontLeftRotation,
                                          wom::PIDConfig<units::radians_per_second, units::volt>(""),
                                          wom::PIDConfig<units::meters_per_second, units::volt>(""),
                                          wom::PIDConfig<units::radian, units::radians_per_second>(""),
                                          wom::PIDConfig<units::meter, units::meters_per_second>(""),
                                          units::meter_t{0.05},
                                          wom::SwerveModuleName::FrontLeft,
                                          ""};

  wom::SwerveModule frontLeft = wom::SwerveModule(frontLeftConfig, wom::SwerveModuleState::kIdle);

  WPI_TalonFX                 frontRightMovementMotor{6, "Drivebase"};
  wom::MotorVoltageController frontRightMovementVoltageController{&frontRightMovementMotor};
  wom::TalonFXEncoder         frontRightMovementEncoder{&frontRightMovementMotor};
  wom::Gearbox frontRightMovement{&frontRightMovementVoltageController, &frontRightMovementEncoder,
                                  frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX                 frontRightRotationMotor{5, "Drivebase"};
  wom::MotorVoltageController frontRightRotationVoltageController{&frontRightMovementMotor};
  wom::CanEncoder             frontRightRotationEncoder{16};
  wom::Gearbox frontRightRotation{&frontRightMovementVoltageController, &frontRightMovementEncoder,
                                  frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  wom::SwerveModuleConfig frontRightConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                           frontLeftMovement,
                                           frontLeftRotation,
                                           wom::PIDConfig<units::radians_per_second, units::volt>(""),
                                           wom::PIDConfig<units::meters_per_second, units::volt>(""),
                                           wom::PIDConfig<units::radian, units::radians_per_second>(""),
                                           wom::PIDConfig<units::meter, units::meters_per_second>(""),
                                           units::meter_t{0.05},
                                           wom::SwerveModuleName::FrontRight,
                                           ""};

  wom::SwerveModule frontRight = wom::SwerveModule(frontRightConfig, wom::SwerveModuleState::kIdle);

  WPI_TalonFX                 backLeftMovementMotor{2, "Drivebase"};
  wom::MotorVoltageController backLeftMovementVoltageController{&backLeftMovementMotor};
  wom::TalonFXEncoder         backLeftMovementEncoder{&backLeftMovementMotor};
  wom::Gearbox                backLeftMovement{&backLeftMovementVoltageController, &backLeftMovementEncoder,
                                frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX                 backLeftRotationMotor{1, "Drivebase"};
  wom::MotorVoltageController backLeftRotationVoltageController{&backLeftMovementMotor};
  wom::CanEncoder             backLeftRotationEncoder{19};
  wom::Gearbox                backLeftRotation{&backLeftMovementVoltageController, &backLeftMovementEncoder,
                                frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  wom::SwerveModuleConfig backLeftConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                         frontLeftMovement,
                                         frontLeftRotation,
                                         wom::PIDConfig<units::radians_per_second, units::volt>(""),
                                         wom::PIDConfig<units::meters_per_second, units::volt>(""),
                                         wom::PIDConfig<units::radian, units::radians_per_second>(""),
                                         wom::PIDConfig<units::meter, units::meters_per_second>(""),
                                         units::meter_t{0.05},
                                         wom::SwerveModuleName::BackLeft,
                                         ""};

  wom::SwerveModule backLeft = wom::SwerveModule(backLeftConfig, wom::SwerveModuleState::kIdle);

  WPI_TalonFX                 backRightMovementMotor{4, "Drivebase"};
  wom::MotorVoltageController backRightMovementVoltageController{&backRightMovementMotor};
  wom::TalonFXEncoder         backRightMovementEncoder{&backRightMovementMotor};
  wom::Gearbox backRightMovement{&backRightMovementVoltageController, &backRightMovementEncoder,
                                 frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  WPI_TalonFX                 backRightRotationMotor{3, "Drivebase"};
  wom::MotorVoltageController backRightRotationVoltageController{&backRightMovementMotor};
  wom::CanEncoder             backRightRotationEncoder{17};
  wom::Gearbox backRightRotation{&backRightMovementVoltageController, &backRightRotationEncoder,
                                 frc::DCMotor::Falcon500(1).WithReduction(6.75)};

  wom::SwerveModuleConfig backRightConfig{frc::Translation2d{units::meter_t{1}, units::meter_t{1}},
                                          frontLeftMovement,
                                          frontLeftRotation,
                                          wom::PIDConfig<units::radians_per_second, units::volt>("", wom::PIDConfig<units::radians_per_second, units::volt>::kp_t{0.5}, wom::PIDConfig<units::radians_per_second, units::volt>::ki_t{0.5}, wom::PIDConfig<units::radians_per_second, units::volt>::kd_t{0.5}),
                                          wom::PIDConfig<units::meters_per_second, units::volt>("", wom::PIDConfig<units::meters_per_second, units::volt>::kp_t{0.5}, wom::PIDConfig<units::meters_per_second, units::volt>::ki_t{0.5}, wom::PIDConfig<units::meters_per_second, units::volt>::kd_t{0.5}),
                                          wom::PIDConfig<units::radian, units::radians_per_second>(""),
                                          wom::PIDConfig<units::meter, units::meters_per_second>(""),
                                          units::meter_t{0.05},
                                          wom::SwerveModuleName::BackRight,
                                          ""};

  wom::SwerveModule backRight = wom::SwerveModule(backRightConfig, wom::SwerveModuleState::kIdle);

  wom::SwerveConfig swerveConfig{frontLeft, frontRight, backLeft, backRight};

};
