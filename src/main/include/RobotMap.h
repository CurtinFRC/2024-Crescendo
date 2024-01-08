// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/Compressor.h>

#include <ctre/Phoenix.h>
#include "drivetrain/SwerveDrive.h"
#include <frc/DoubleSolenoid.h>
#include <units/length.h>

#include <iostream>
#include <string>

#include <units/angle.h>

#include <frc/system/plant/DCMotor.h>

#include "Wombat.h"

struct RobotMap {
  struct Controllers {
    frc::XboxController driver         = frc::XboxController(0);
    frc::XboxController coDriver       = frc::XboxController(1);
    frc::XboxController testController = frc::XboxController(2);
  };
  Controllers controllers;

  struct SwerveBase{
    CANCoder frontLeftCancoder{19};
    CANCoder frontRightCancoder{17};
    CANCoder backLeftCancoder{16};
    CANCoder backRightCancoder{18};

    wom::NavX gyro;
    wpi::array<WPI_TalonFX*, 4> turnMotors{
      new WPI_TalonFX(7, "Drivebase"), new WPI_TalonFX(5, "Drivebase"), new WPI_TalonFX(1, "Drivebase"), new WPI_TalonFX(3, "Drivebase")
    };
    wpi::array<WPI_TalonFX*, 4> driveMotors{
      new WPI_TalonFX(9, "Drivebase"), new WPI_TalonFX(6, "Drivebase"), new WPI_TalonFX(2, "Drivebase"), new WPI_TalonFX(4, "Drivebase")
    };

    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
      wom::SwerveModuleConfig{ // front left module
        frc::Translation2d(10.761_in, 9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[0]),
          new wom::TalonFXEncoder(driveMotors[0], 6.75),
          frc::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[0]),
          new wom::CanEncoder(19, 4096, 12.8),
          frc::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &frontLeftCancoder,
        4_in / 2
      },
      wom::SwerveModuleConfig{ // front right module
        frc::Translation2d(10.761_in, -9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[1]),
          new wom::TalonFXEncoder(driveMotors[1], 6.75),
          frc::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[1]),
          new wom::CanEncoder(17, 4096, 12.8),
          frc::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &frontRightCancoder,
        4_in / 2
      },
      wom::SwerveModuleConfig{ // back left module
        frc::Translation2d(-10.761_in, 9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[2]),
          new wom::TalonFXEncoder(driveMotors[2], 6.75),
          frc::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[2]),
          new wom::CanEncoder(16, 4096, 12.8),
          frc::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &backRightCancoder,
        4_in / 2
      },
      wom::SwerveModuleConfig{ // back right module
        frc::Translation2d(-10.761_in, -9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[3]),
          new wom::TalonFXEncoder(driveMotors[3], 6.75),
          frc::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[3]),
          new wom::CanEncoder(18, 4096, 12.8),
          frc::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &backLeftCancoder,
        4_in / 2
      },
    };

    // Setting the PID path and values to be used for SwerveDrive and SwerveModules
    wom::SwerveModule::angle_pid_conf_t anglePID {
      "/drivetrain/pid/angle/config",
      2_V / 360_deg,
      0.0_V / (100_deg * 1_s),
      0_V / (100_deg / 1_s),
      1_deg,
      0.5_deg / 2_s
    };
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
      "/drivetrain/pid/velocity/config",
      //  12_V / 4_mps // webers per metre
    };
    wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID {
      "/drivetrain/pid/pose/angle/config",
      180_deg / 1_s / 45_deg,
      wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0.1},
      0_deg / 1_deg,
      10_deg,
      10_deg / 1_s
    };
    wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
      "/drivetrain/pid/pose/position/config",
      3_mps / 1_m,
      wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0.15},
      0_m / 1_m,
      20_cm, 
      10_cm / 1_s,
      10_cm
    };

    // the config for the whole swerve drive
    wom::SwerveDriveConfig config{
      "/drivetrain",
      anglePID, velocityPID,
      moduleConfigs,// each module
      &gyro,
      poseAnglePID, 
      posePositionPID,
      60_kg, // robot mass (estimate rn)
      {0.1, 0.1, 0.1},
      {0.9, 0.9, 0.9}
    };  

    // current limiting and setting idle mode of modules to brake mode
    SwerveBase() {
      for (size_t i = 0; i < 4; i++) {
        turnMotors[i]->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 15, 15, 0));
        driveMotors[i]->SetNeutralMode(NeutralMode::Brake);
        turnMotors[i]->SetNeutralMode(NeutralMode::Brake);
        driveMotors[i]->SetInverted(true);
      }
    }
  };
  SwerveBase swerveBase;


};
