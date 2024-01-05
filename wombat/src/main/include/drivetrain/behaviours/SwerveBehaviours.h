// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "utils/Pathplanner.h"

#include "utils/Pathplanner.h"
#include "utils/Util.h"

#include <frc/smartdashboard/Field2d.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/SendableChooser.h"
#include <frc/smartdashboard/Field2d.h>

#include <frc/Timer.h>

#include <string>

namespace wom {
namespace drivetrain {
  namespace behaviours {

    class FieldRelativeSwerveDrive : public behaviour::Behaviour {
     public:
      FieldRelativeSwerveDrive(wom::drivetrain::Swerve *swerve, frc::XboxController &driver);

      void OnTick(units::second_t dt) override;

     private:
      wom::drivetrain::Swerve *_swerve;
      frc::XboxController     &_driver;
        
    };

    class GoToPose : public behaviour::Behaviour {
      public:
        GoToPose(wom::drivetrain::Swerve *swerve, frc::Pose3d pose);

        void OnTick(units::second_t dt) override;

      private:
        wom::drivetrain::Swerve *_swerve;
        frc::Pose3d _pose;
    };

    class FollowTrajectory : public behaviour::Behaviour {
      public:
        FollowTrajectory(wom::drivetrain::Swerve *swerve, wom::utils::Pathplanner *pathplanner, std::string path);

        void OnTick(units::second_t dt) override;

        void OnStart() override;

      private:
        wom::utils::Pathplanner *_pathplanner;
        std::string _path;
        wom::drivetrain::Swerve *_swerve;
        frc::Trajectory _trajectory;
        frc::Timer m_timer;
    };

    

    class TempSimSwerveDrive {
     public:
      TempSimSwerveDrive(frc::Timer *timer, frc::Field2d *field);

      void OnUpdate();

      void SetPath(std::string path);

      frc::Pose3d GetPose();

     private:

      frc::sim::DifferentialDrivetrainSim m_driveSim{
            frc::DCMotor::NEO(2), // 2 NEO motors on each side of the drivetrain.
            7.29,               // 7.29:1 gearing reduction.
            7.5_kg_sq_m,        // MOI of 7.5 kg m^2 (from CAD model).
            60_kg,              // The mass of the robot is 60 kg.
            3_in,               // The robot uses 3" radius wheels.
            0.7112_m,           // The track width is 0.7112 meters.

            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};

      wom::utils::Pathplanner m_pathplanner;

      frc::Trajectory current_trajectory;
      
      std::shared_ptr<nt::NetworkTable> current_trajectory_table;
      std::shared_ptr<nt::NetworkTable> current_trajectory_state_table;

      frc::Timer *m_timer;

      frc::Field2d *m_field;

      std::string m_path;
    };

    class AutoSwerveDrive {
      public:
        AutoSwerveDrive(wom::drivetrain::Swerve *swerve, frc::Timer *timer, frc::Field2d *field);
  
        void OnUpdate();
  
        void SetPath(std::string path);
  
      private:
        wom::drivetrain::Swerve *_swerve;

        TempSimSwerveDrive *_simSwerveDrive;
  
        frc::Timer *m_timer;
  
        frc::Field2d *m_field;
  
        std::string m_path;
      };
  }  // namespace behaviours
}  // namespace drivetrain
}  // namespace wom
