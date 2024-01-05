#pragma once

#include "Wombat.h"
#include "tank.h"
#include <frc/XboxController.h>

struct RobotMap {
  frc::XboxController driver{0};

  struct ArmSystem {
    rev::CANSparkMax ArmMotor{98, rev::CANSparkMax::MotorType::kBrushless};
    wom::VoltageController ArmMotorGroup = wom::VoltageController::Group(ArmMotor);

    wom::Gearbox ArmGearbox {
      &ArmMotorGroup,
      nullptr,
      wom::DCMotor::NEO(1).WithReduction(1)
    };

    ArmConfig config {
      ArmGearbox,
    };

  }; ArmSystem armSystem;
  
  struct TankSystem {
    wom::VoltageController tankDriveMotorFrontLeft{ new WPI_TalonSRX(99)};
    wom::VoltageController tankDriveMotorFrontRight{ new WPI_TalonSRX(99)};
    wom::VoltageController tankDriveMotorBackLeft{ new WPI_TalonSRX(99)};
    wom::VoltageController tankDriveMotorBackRight{ new WPI_TalonSRX(99)};

    //wom::VoltageController tankDriveMotorGroupLeft = wom::VoltageController::Group(new WPI_TalonSRX(99), new WPI_TalonSRX(99));
   // wom::VoltageController tankDriveMotorGroupRight = wom::VoltageController::Group(tankDriveMotorBackRight, tankDriveMotorFrontRight);

    wom::Gearbox tankFrontLeftGearbox {
      &tankDriveMotorFrontLeft,
      nullptr,
      wom::DCMotor::CIM(1).WithReduction(1)
    };

    wom::Gearbox tankBackLeftGearbox {
      &tankDriveMotorBackLeft,
      nullptr,
      wom::DCMotor::CIM(1).WithReduction(1)
    };

    wom::Gearbox tankFrontRightGearbox {
      &tankDriveMotorFrontRight,
      nullptr,
      wom::DCMotor::CIM(1).WithReduction(1)
    };
    
    wom::Gearbox tankBackRightGearbox {
      &tankDriveMotorBackRight,
      nullptr,
      wom::DCMotor::CIM(1).WithReduction(1)
    };

    TankConfig tankConfig {
      tankFrontRightGearbox,
      tankBackRightGearbox,
      tankFrontLeftGearbox,
      tankBackLeftGearbox
    };
  }; TankSystem tankSystem;

  //port to be filled later.

};