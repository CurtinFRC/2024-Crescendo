// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

// include units
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

// include frc::DriveKinematics
#include <frc/kinematics/DifferentialDriveKinematics.h>

// include frc::RamseteController
#include <frc/controller/RamseteController.h>

// include frc::Timer
#include <frc/Timer.h>

#include "Wombat.h"


void Robot::RobotInit() {

    
    m_chooser.SetDefaultOption("Default Auto", "Default Auto");

    frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

    m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

    m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
    m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

    frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

    frc::SmartDashboard::PutData("Field", &m_field);

    simulation_timer = frc::Timer();

    wom::BehaviourScheduler::GetInstance()->Register(&robotmap.swerve);
    robotmap.swerve.SetDefaultBehaviour([this]() {
      return wom::make<wom::ManualDrivebase>(&robotmap.swerve, robotmap.controllers.driver);
    });

    m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
    //m_driveSim = wom::TempSimSwerveDrive();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_driveSim->SetPath(m_path_chooser.GetSelected());
}
void Robot::AutonomousPeriodic() {
    m_driveSim->OnUpdate();
}

void Robot::TeleopInit() {
  sched = wom::BehaviourScheduler::GetInstance();
  sched->InterruptAll();
  sched->Schedule(wom::make<wom::ManualDrivebase>(&robotmap.swerve, robotmap.controllers.driver));
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
    
}

void Robot::SimulationPeriodic() {
}
