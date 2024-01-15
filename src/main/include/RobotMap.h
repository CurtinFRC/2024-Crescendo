#pragma once

#include "Wombat.h"
#include "Intake.h"
#include "IntakeBehaviour.h"
#include <frc/XboxController.h>

struct RobotMap {
  frc::XboxController driver{0};

  struct IntakeSystem {

    //rev::CANSparkMax IntakeController{99,rev::CANSparkMax::MotorType::kBrushless};
    wom::VoltageController IntakeMotor{new rev::CANSparkMax(99, rev::CANSparkMax::MotorType::kBrushless) };
    frc::DigitalInput intakeSensor(0);

    //wom::VoltageController IntakeMotor{ IntakeController NEO(99)};

    wom::Gearbox IntakeGearbox {
      &IntakeMotor,
      nullptr,
      wom::DCMotor::NEO(1).WithReduction(1)
    };

    IntakeConfig config {
      IntakeGearbox,
    };
  }; IntakeSystem intakeSystem;

  //port to be filled later.

};