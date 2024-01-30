
#pragma once

#include "Wombat.h"

#include "AlphaArmBehaviour.h"
#include "Intake.h"
#include "Shooter.h"

namespace autos {
  std::shared_ptr<behaviour::Behaviour> AutoTest(wom::drivetrain::SwerveDrive _swerveDrive, Shooter _shooter, Intake _intake, AlphaArm _alphaArm);

  // std::shared_ptr<behaviour::Behaviour> Taxi(Drivebase driveBase, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleClose(Drivebase driveBase, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleFar(Drivebase driveBase, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleCloseDoubleFar(Drivebase driveBase, Shooter shooter, Mag mag, Intake intake, Arm _arm);

  // std::shared_ptr<behaviour::Behaviour> QuadrupleCloseSingleFar(Drivebase driveBase, Shooter shooter, Mag mag, Intake intake, Arm _arm);
}