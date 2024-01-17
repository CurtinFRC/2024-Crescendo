#pragma once

#include "Wombat.h"
#include "Intake.h"
#include "IntakeBehaviour.h"
#include <frc/XboxController.h>

struct RobotMap {
  frc::XboxController driver{0};

  struct IntakeSystem {
    rev::CANSparkMax intakeMotor{99, rev::CANSparkMax::MotorType::kBrushless};
    wom::VoltageController intakeMotorGroup = wom::VoltageController::Group(intakeMotor);
    wom::CANSparkMaxEncoder intakeEncoder{&intakeMotor, 42};
    frc::DigitalInput intakeSensor {0};
    frc::DigitalInput magSensor {0};
    frc::DigitalInput shooterSensor {0};

    wom::Gearbox IntakeGearbox { 
      &intakeMotorGroup,
      &intakeEncoder,
      wom::DCMotor::NEO(1).WithReduction(1)
    };

    IntakeConfig config {
      IntakeGearbox,
      &intakeSensor,
      &magSensor,
      &shooterSensor
    };
  }; IntakeSystem intakeSystem;

};