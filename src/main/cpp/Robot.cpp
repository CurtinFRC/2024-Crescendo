#include "Robot.h"
// include units
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

void Robot::RobotInit() {
    m_chooser.SetDefaultOption("Default Auto", "Default Auto");

    frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

    frc::SmartDashboard::PutData("Field", &m_field);

}
void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("FMSInfo");

    // put table->GetEntry("StationNumber").GetInteger(-1) into smart dashboard

    switch (table->GetEntry("StationNumber").GetInteger(1)) {
        case 1:
            frc::SmartDashboard::PutNumber("StationNumber", table->GetEntry("StationNumber").GetInteger(-1));

            m_driveSim.SetPose(frc::Pose2d(1_m, 1_m, frc::Rotation2d(0_deg)));
            break;
        case 2:
            frc::SmartDashboard::PutNumber("StationNumber", table->GetEntry("StationNumber").GetInteger(-1));

            m_driveSim.SetPose(frc::Pose2d(1_m, 2_m, frc::Rotation2d(0_deg)));
            break;
        case 3:
            frc::SmartDashboard::PutNumber("StationNumber", table->GetEntry("StationNumber").GetInteger(-1));

            m_driveSim.SetPose(frc::Pose2d(1_m, 3_m, frc::Rotation2d(0_deg)));
            break;

    }
}

void Robot::AutonomousPeriodic() {
    m_field.SetRobotPose(m_driveSim.GetPose());

    frc::Trajectory trajectory = m_pathplanner.getTrajectory("output/Path1.wpilib.json");

    // create a netowrk table for the trajectory
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("trajectory");

    // write the trajectory to the network table
    wom::utils::WriteTrajectory(table, trajectory);

    m_driveSim.Update(20_ms);
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {

}
void Robot::SimulationPeriodic() {}
