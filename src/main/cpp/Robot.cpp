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

    frc::SmartDashboard::PutData("Field", &m_field);

    simulation_timer = frc::Timer();
}
void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("FMSInfo");

    current_trajectory = m_pathplanner.getTrajectory("paths/output/Path1.wpilib.json");

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

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
