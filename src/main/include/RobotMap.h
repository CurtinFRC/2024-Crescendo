#pragma once


#include "intake.h"
#include "tank.h"
#include <frc/XboxController.h>

struct RobotMap {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};

    struct Intake {
        WPI_TalonSRX intakeMotor{99};

        wom::VoltageController intakeMotorGroup = wom::VoltageController::Group(intakeMotor);

        
        wom::Gearbox intakeGearbox {
            &intakeMotorGroup,
            nullptr,
            
            wom::DCMotor::NEO(1).WithReduction(1)
        };

        IntakeConfig config {
            intakeGearbox
        };
    }; Intake intakeSystem;
    
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
    
};
    
    
