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

};
  