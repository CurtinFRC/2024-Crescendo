#pragma once

#include "Wombat.h"

#include "AlphaArmBehaviour.h"
#include "IntakeBehaviour.h"
#include "ShooterBehaviour.h"


namespace autos {
  std::shared_ptr<behaviour::Behaviour> AutoTest(wom::drivetrain::SwerveDrive* _swerveDrive, frc::Timer* timer, frc::Field2d* field, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

  std::shared_ptr<behaviour::Behaviour> Taxi(wom::drivetrain::SwerveDrive* _swerveDrive, frc::Timer* timer, frc::Field2d* field, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

  std::shared_ptr<behaviour::Behaviour> QuadrupleClose(wom::drivetrain::SwerveDrive* _swerveDrive, frc::Timer* timer, frc::Field2d* field, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

  std::shared_ptr<behaviour::Behaviour> QuadrupleFar(wom::drivetrain::SwerveDrive* _swerveDrive, frc::Timer* timer, frc::Field2d* field, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

  std::shared_ptr<behaviour::Behaviour> QuadrupleCloseDoubleFar(wom::drivetrain::SwerveDrive* _swerveDrive, frc::Timer* timer, frc::Field2d* field, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);

  std::shared_ptr<behaviour::Behaviour> QuadrupleCloseSingleFar(wom::drivetrain::SwerveDrive* _swerveDrive, frc::Timer* timer, frc::Field2d* field, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm);
}