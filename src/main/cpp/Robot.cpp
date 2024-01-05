// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"
#include "intakeBehaviour.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {

 lastPeriodic = wom::now();
 shooter = new Shooter(map.shooterSystem.config);
 wom::BehaviourScheduler::GetInstance()->Register(shooter);
 shooter->SetDefaultBehaviour([this] () {
    return wom::make<ShooterManualControl>(shooter, map.codriver);
   
 });
}
intake = new Intake(map.intakeSystem.config);
    wom::BehaviourScheduler::GetInstance()->Register(intake);
    intake->SetDefaultBehaviour([this]() {
        return wom::make<IntakeManualControl>(intake, map.codriver);
      
     tank = new TankDrive(map.tankSystem.tankConfig);
     wom::BehaviourScheduler::GetInstance()->Register(tank);


   tank->SetDefaultBehaviour([this]() {
    return wom::make<TankManualControl>(tank, map.driver);
  });
    });
void Robot::RobotPeriodic() {
 units::second_t dt = wom::now() - lastPeriodic;
 lastPeriodic = wom::now();

 loop.Poll();
 wom::BehaviourScheduler::GetInstance()->Tick();

 shooter->OnUpdate(dt);
 intake->OnUpdate(dt);
 tank->OnUpdate(dt);
  
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    loop.Clear();
    wom::BehaviourScheduler *Scheduler = wom::BehaviourScheduler::GetInstance();
    Scheduler->InterruptAll();


}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
