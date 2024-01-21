#pragma once
#include "Shooter.h"

#include <frc/XboxController.h>

struct RobotMap {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};

    struct Shooter {
        rev::CANSparkMax shooterMotor{99, rev::CANSparkMax::MotorType::kBrushless};
        frc::DigitalInput shooterSensor{2};

        wom::VoltageController shooterMotorGroup = wom::VoltageController::Group(shooterMotor);
        wom::CANSparkMaxEncoder shooterEncoder=wom::CANSparkMaxEncoder(&shooterMotor);

        wom::Gearbox shooterGearbox {
            &shooterMotorGroup,

            &shooterEncoder,

            wom::DCMotor::NEO(1).WithReduction(1)
        };

        ShooterConfig config {
            "shooterGearbox",
            shooterGearbox,
            wom::PIDConfig<units::radians_per_second, units::volt>  {
                "/Shooter/shooter/velocityPID/config",
                9_V / (180_deg / 1_s),
                0_V / 25_deg,
                0_V / (90_deg / 1_s / 1_s)
            },
            &shooterSensor,
        };

    }; Shooter shooterSystem;
}; 