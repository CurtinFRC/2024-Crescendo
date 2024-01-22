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

#include "Intake.h"
#include "IntakeBehaviour.h"
#include "Wombat.h"

struct RobotMap {
  struct Controllers {
    frc::XboxController driver = frc::XboxController(0);
    frc::XboxController codriver = frc::XboxController(1);
  };
  Controllers controllers;

  struct IntakeSystem {
    rev::CANSparkMax intakeMotor{2, rev::CANSparkMax::MotorType::kBrushed};
    // wom::CANSparkMaxEncoder intakeEncoder{&intakeMotor, 0.1_m};
    // frc::DigitalInput intakeSensor{0};
    // frc::DigitalInput magSensor{0};
    // frc::DigitalInput shooterSensor{0};

    wom::Gearbox IntakeGearbox{&intakeMotor, nullptr, frc::DCMotor::CIM(1)};

    IntakeConfig config{IntakeGearbox /*, &intakeSensor, &magSensor, &shooterSensor*/};
  };
  IntakeSystem intakeSystem;
};
