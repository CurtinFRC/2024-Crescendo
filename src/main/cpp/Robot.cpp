#include "Robot.h"

void Robot::RobotInit() {
    m_chooser.SetDefaultOption("Default Auto", "Default Auto");

    frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

    frc::SmartDashboard::PutData("Field", &m_field);

}
void Robot::RobotPeriodic() {
    //m_field.SetRobotPose(m_odometry.GetPose());

    
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {

}
void Robot::SimulationPeriodic() {}