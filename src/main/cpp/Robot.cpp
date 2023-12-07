#include "Robot.h"

void Robot::RobotInit() {
  BehaviourScheduler::GetInstance()->Register(&robotmap.swerve);
  robotmap.swerve.SetDefaultBehaviour([this]() {
    return make<FieldRelativeSwerveDrive>(&robotmap.swerve, robotmap.controllers.driver);
  });
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll();
  sched->Schedule(make<FieldRelativeSwerveDrive>(&robotmap.swerve, robotmap.controllers.driver));
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
