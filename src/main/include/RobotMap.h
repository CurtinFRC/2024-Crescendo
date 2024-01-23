// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project
#pragma once

#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/XboxController.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/length.h>

#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Shooter.h"
#include "Wombat.h"

struct RobotMap {
  struct Controllers {
    frc::XboxController driver = frc::XboxController(0);
    frc::XboxController codriver = frc::XboxController(1);
    frc::XboxController testController = frc::XboxController(2);
  };
  Controllers controllers;

  struct Shooter {
    rev::CANSparkMax shooterMotor{11, rev::CANSparkMax::MotorType::kBrushless};
    // frc::DigitalInput shooterSensor{2};

    // wom::VoltageController shooterMotorGroup = wom::VoltageController::Group(shooterMotor);
    // wom::CANSparkMaxEncoder* shooterEncoder = new wom::CANSparkMaxEncoder(&shooterMotor, 0.01_m);
    wom::Gearbox shooterGearbox{&shooterMotor, nullptr, frc::DCMotor::NEO(1)};

    ShooterConfig config{
        "shooterGearbox",
        shooterGearbox,
        // &shooterSensor,
    };
  };
  Shooter shooterSystem;
  
  struct SwerveBase {
    ctre::phoenix6::hardware::CANcoder frontLeftCancoder{18, "Drivebase"};
    ctre::phoenix6::hardware::CANcoder frontRightCancoder{19, "Drivebase"};
    ctre::phoenix6::hardware::CANcoder backLeftCancoder{16, "Drivebase"};
    ctre::phoenix6::hardware::CANcoder backRightCancoder{17, "Drivebase"};

    ctre::phoenix6::hardware::Pigeon2* gyro =
        new ctre::phoenix6::hardware::Pigeon2(20, "Drivebase");
    wpi::array<ctre::phoenix6::hardware::TalonFX*, 4> turnMotors{
        new ctre::phoenix6::hardware::TalonFX(7, "Drivebase"),   // front left
        new ctre::phoenix6::hardware::TalonFX(2, "Drivebase"),   // front right
        new ctre::phoenix6::hardware::TalonFX(6, "Drivebase"),   // back left
        new ctre::phoenix6::hardware::TalonFX(4, "Drivebase")};  // back right
    wpi::array<ctre::phoenix6::hardware::TalonFX*, 4> driveMotors{
        new ctre::phoenix6::hardware::TalonFX(9, "Drivebase"),   // front left
        new ctre::phoenix6::hardware::TalonFX(1, "Drivebase"),   // front right
        new ctre::phoenix6::hardware::TalonFX(5, "Drivebase"),   // back left
        new ctre::phoenix6::hardware::TalonFX(3, "Drivebase")};  // back right

    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
        wom::SwerveModuleConfig{ //CORRECT
            // front left module
            frc::Translation2d(10.761_in, 9.455_in),
            wom::Gearbox{
                driveMotors[0],
                new wom::TalonFXEncoder(driveMotors[0], 0.0445_m, 6.75),
                frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[0],
                         new wom::CanEncoder(18, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &frontLeftCancoder, 4_in / 2},
        wom::SwerveModuleConfig{ //CORRECT
            // front right module
            frc::Translation2d(10.761_in, -9.455_in),
            wom::Gearbox{
                driveMotors[1],
                new wom::TalonFXEncoder(driveMotors[1], 0.0445_m, 6.75),
                frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[1],
                         new wom::CanEncoder(19, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &frontRightCancoder, 4_in / 2},
        wom::SwerveModuleConfig{
            // back left module
            frc::Translation2d(-10.761_in, 9.455_in),
            wom::Gearbox{
                driveMotors[2],
                new wom::TalonFXEncoder(driveMotors[2], 0.0445_m, 6.75),
                frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[2],
                         new wom::CanEncoder(16, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &backRightCancoder, 4_in / 2},
        wom::SwerveModuleConfig{
            // back right module
            frc::Translation2d(-10.761_in, -9.455_in),
            wom::Gearbox{
                driveMotors[3],
                new wom::TalonFXEncoder(driveMotors[3], 0.0445_m, 6.75),
                frc::DCMotor::Falcon500(1).WithReduction(6.75)},
            wom::Gearbox{turnMotors[3],
                         new wom::CanEncoder(17, 0.0445_m, 4096, 12.8),
                         frc::DCMotor::Falcon500(1).WithReduction(12.8)},
            &backLeftCancoder, 4_in / 2},
    };

    // Setting the PID path and values to be used for SwerveDrive and
    // SwerveModules
    /*wom::SwerveModule::angle_pid_conf_t anglePID{
        "/drivetrain/pid/angle/config", 90_V / 360_deg, 0.0_V / (100_deg * 1_s),
        0_V / (100_deg / 1_s)};*/
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
        "/drivetrain/pid/velocity/config",
        //  12_V / 4_mps // webers per metre
    };
    /*wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID{
        "/drivetrain/pid/pose/angle/config",
        0_deg / 1_s / 45_deg,
        wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0},
        0_deg / 1_deg};*/
    wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
        "/drivetrain/pid/pose/position/config",
        0_mps / 1_m,
        wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0.15},
        0_m / 1_m,
        0_cm};

    // the config for the whole swerve drive
    wom::SwerveDriveConfig config{"/drivetrain",
                                  //anglePID,
                                  velocityPID,
                                  moduleConfigs,  // each module
                                  gyro,
                                  //poseAnglePID,
                                  posePositionPID,
                                  60_kg,  // robot mass (estimate rn)
                                  {0.1, 0.1, 0.1},
                                  {0.9, 0.9, 0.9}};

    // current limiting and setting idle mode of modules to brake mode
    // SwerveBase() {
    //  for (size_t i = 0; i < 4; i++) {
    //    turnMotors[i]->ConfigSupplyCurrentLimit(
    //        SupplyCurrentLimitConfiguration(true, 15, 15, 0));
    //    driveMotors[i]->SetNeutralMode(NeutralMode::Brake);
    //    turnMotors[i]->SetNeutralMode(NeutralMode::Brake);
    //    driveMotors[i]->SetInverted(true);
    //  }
    //}
  };
  SwerveBase swerveBase;

  struct SwerveTable {
    std::shared_ptr<nt::NetworkTable> swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");
  }; SwerveTable swerveTable;
};
