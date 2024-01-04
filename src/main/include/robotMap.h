#pragma once

#include "arm.h"
#include <rev/CANSparkMax.h>

#include <frc/XboxController.h>

struct RobotMap {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};

    struct Arm {
        rev::CANSparkMax armMotor{99, rev::CANSparkMax::MotorType::kBrushless};

        wom::VoltageController armMotorGroup = wom::VoltageController::Group(armMotor);

        // wom::CANSparkMaxEncoder armEncoder(&armMotor, 100);

        wom::Gearbox armGearBox {
            &armMotorGroup,
            // &armEncoder,
            nullptr,
            wom::DCMotor::NEO(1).WithReduction(1)
        };

        ArmConfig config {
            armGearBox
        };
    }; Arm armSystem;
};