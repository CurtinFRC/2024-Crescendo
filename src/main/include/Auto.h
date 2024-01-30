
#pragma once

#include "Wombat.h"
#include "Intake.h"
#include "Shooter.h"
#include "AlphaArm.h"

namespace autos {
  std::shared_ptr<behaviour::Behaviour> AutoTest(wom::drivetrain::SwerveDrive Swervedrive, wom::subsystems::Shooter shooter, Intake intake, wom::subsystems::Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> Taxi(SwerveDrive Swervedrive, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleClose(SwerveDrive Swervedrive, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleFar(SwerveDrive Swervedrive, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleCloseDoubleFar(SwerveDrive Swervedrive, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleCloseSingleFar(SwerveDrive Swervedrive, Shooter shooter, Mag mag, Intake intake, Arm _arm);
}


