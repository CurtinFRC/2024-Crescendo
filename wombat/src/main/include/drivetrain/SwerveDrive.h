#pragma once

#include <string>
#include <iostream>

#include <units/charge.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include <networktables/DoubleTopic.h>

#include "behaviour/HasBehaviour.h"
#include "utils/PID.h"
#include "utils/Gearbox.h"
#include "utils/Util.h"
#include "vision/Limelight.h"

namespace wom {
namespace drivetrain {
    enum SwerveModuleName {
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight,
    };
    

    struct SwerveModuleConfig {
      wom::utils::Gearbox rotationGearbox;
      wom::utils::Gearbox movementGearbox;

      wom::utils::PIDConfig<units::radians_per_second, units::volt> rotationalVelocityPID;
      wom::utils::PIDConfig<units::meters_per_second, units::volt> movementVelocityPID;
      
      wom::utils::PIDConfig<units::radian, units::radians_per_second> rotationPID;
      wom::utils::PIDConfig<units::meter, units::meters_per_second> movementPID;

      units::meter_t wheelRadius;

      nt::DoubleTopic angularVelocityTopic;
      nt::DoubleTopic velocityTopic;

      wom::vision::Limelight vision;

      std::string path;
      SwerveModuleName name;
    };

    enum class SwerveModuleState {
        kIdle,
        kCalibration,
        kPID,
    };

    class SwerveModule : public behaviour::HasBehaviour {
      nt::DoublePublisher angularVelocityPublisher;
      nt::DoublePublisher velocityPublisher;

     public:
      SwerveModule(SwerveModuleConfig config, SwerveModuleState state);

      SwerveModuleConfig GetConfig();
      SwerveModuleState GetState();

      void SetState(SwerveModuleState state);

      void Log();
      units::meters_per_second_t GetSpeed();
      void PIDControl(units::second_t dt, units::radian_t rotation, units::meter_t movement);

      void OnStart();
      void OnUpdate(units::second_t dt, units::radian_t rotation, units::meter_t movement);

     protected:
      wom::utils::PIDController<units::radians_per_second, units::volt> _rotationalVelocityPID;
      wom::utils::PIDController<units::meters_per_second, units::volt> _movementVelocityPID;

      wom::utils::PIDController<units::radian, units::radians_per_second> _rotationalPID;
      wom::utils::PIDController<units::meters, units::meters_per_second> _movementPID;

     private:
      units::volt_t voltageRotation;
      units::volt_t voltageMovement;

      units::meters_per_second_t velocity;
      units::radians_per_second_t angularVelocity;

      SwerveModuleConfig _config;
      SwerveModuleState _state;

    };
}
};
