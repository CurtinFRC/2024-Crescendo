#pragma once

#include "Shooter.h"
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/XboxController.h>

struct RobotMap {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};

    struct Shooter {
        WPI_TalonSRX launcherMotor{7};
        WPI_TalonSRX loaderMotor{6};

        wom::DutyCycleEncoder launcherEncoder{0, 2048};

        wom::VoltageController launcherMotorGroup = wom::VoltageController::Group(launcherMotor);
        wom::VoltageController loaderMotorGroup = wom::VoltageController::Group(loaderMotor);

        wom::Gearbox LauncherGearBox {
            &launcherMotorGroup,
            &launcherEncoder,
            wom::DCMotor::CIM(1).WithReduction(1)
        };

        wom::Gearbox LoaderGearBox {
            &loaderMotorGroup,
            nullptr,
            wom::DCMotor::CIM(1).WithReduction(12)
        };

        ShooterConfig config {
            "Shooter",
            LauncherGearBox,
            LoaderGearBox
        };
    }; Shooter shooterSystem;
};