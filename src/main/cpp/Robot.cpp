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
      return wom::make<wom::FieldRelativeSwerveDrive>(&robotmap.swerve, robotmap.controllers.driver);
    });
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("FMSInfo");

    current_trajectory = m_pathplanner.getTrajectory(m_path_chooser.GetSelected());

     // create a netowrk table for the trajectory
    std::shared_ptr<nt::NetworkTable> trajectory_table = nt::NetworkTableInstance::GetDefault().GetTable("trajectory_path");
    current_trajectory_table = nt::NetworkTableInstance::GetDefault().GetTable("current_trajectory");
    current_trajectory_state_table = nt::NetworkTableInstance::GetDefault().GetTable("current_trajectory_state");

    // write the trajectory to the network table
    wom::utils::WriteTrajectory(trajectory_table, current_trajectory);

    // sample the first trajectory state
    frc::Trajectory::State desired_state = current_trajectory.Sample(0_s);

    // move drivebase position to the desired state
    m_driveSim.SetPose(wom::utils::TrajectoryStateToPose2d(desired_state));

    simulation_timer.Reset();
    simulation_timer.Start();
}
void Robot::AutonomousPeriodic() {
    m_field.SetRobotPose(m_driveSim.GetPose());

    // get the current trajectory state
    frc::Trajectory::State desired_state = current_trajectory.Sample(simulation_timer.Get());

    // get the current pose of the robot
    frc::Pose2d current_pose = m_driveSim.GetPose();
    
    // get the current wheel speeds
    wom::utils::WriteTrajectoryState(current_trajectory_state_table, desired_state);

    // move drivebase position to the desired state
    m_driveSim.SetPose(wom::utils::TrajectoryStateToPose2d(desired_state));
    
    // update the drivebase
    m_driveSim.Update(20_ms);
}

void Robot::TeleopInit() {
  sched = wom::BehaviourScheduler::GetInstance();
  sched->InterruptAll();
  sched->Schedule(wom::make<wom::FieldRelativeSwerveDrive>(&robotmap.swerve, robotmap.controllers.driver));
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
