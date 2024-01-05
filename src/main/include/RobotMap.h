#pragma once

#include "intake.h"
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
};