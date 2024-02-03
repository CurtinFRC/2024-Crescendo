#include "Auto.h"

std::shared_ptr<behaviour::Behaviour> Taxi(wom::drivetrain::SwerveDrive _driveBase, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(_arm, 60)
    <<make<AutoShoot>(_shooter, 12);
    <<make<ArmToSetPoint>(_arm, 0);
    <<make<DriveToLocation>(_driveBase, 12, distance);
}
//Shoots starting note then moves out of starting position.

std::shared_ptr<Behaviour> QuadrupleClose(wom::drivetrain::SwerveDrive _driveBase, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return make<ArmToSetPoint>(_arm, 60) 
    << make<AutoShoot>(_shooter, 12)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoot>(_shooter, 12)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoot>(_shooter, 12)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoo>(_shooter, 12)

     /*
      4N Close
    1. Shoot starting note into speaker
    2. Intake note from close note
    3. Shoot note into speaker
    4. Intake note from close floor note
    5. Shoot note into speaker
    6. Intake not from close floor
    7. Shoot note
    */
}

std::shared_ptr<Behaviour> QuadrupleFar(wom::drivetrain::SwerveDrive _driveBase, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return make<ArmToSetPoint>(_arm, /12)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    //<< make<ArmToSetPoint>(_arm, armAngle)
    //<< make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    //<< make<ArmToSetPoint>(_arm, armAngle)
    //<< make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    adasda
    //<< make<ArmToSetPoint>(_arm, armAngle)
    //<< make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoot>(_shooter, 12)
    // << make<ArmToSetPoint>(_arm, armAngle)
    // << make<DriveToLocation>(_driveBase, 12, distance, direction)   do this if possible
    // << make<ArmToSetPoint>(_arm, armAngle)
    // << make<AutoIntake>(_intake, 12)

    /*
      4N Far
    1. Shoot start note in speaker
    2. Drive to far note
    3. Intake note
    4. Drive back to shooting line
    5. Shoot note into speaker
    6. Drive to note
    7. Intake note
    8. Drive to shooting line
    9. Shoot note
    10. Drive to note
    11. Intake note
    12. Drive to shooting line
    13. Shoot note
    14. Drive to intake note (if possible)
    */
}

std::shared_ptr<Behaviour> QuadrupleCloseDoubleFar(wom::drivetrain::SwerveDrive _driveBase, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<Intake>(_intake, 12)
    //<< make<ArmToSetPoint>(_arm, armAngle)
    //<< make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    //<< make<ArmToSetPoint>(_arm, armAngle)
    //<< make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    //<< make<ArmToSetPoint>(_arm, armAngle)
    //<< make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, armAngle)
    << make<AutoShoot>(_shooter, 12)
    
    //   4N Close 2N Far
    // 1. Shoot note
    // 2. Drive to close note
    // 3. Intake note
    // 4. Shoot note
    // 5. Drive to close note
    // 6. Intake note
    // 7. Shoot note
    // 8. Drive to close note
    // 9. Intake note
    // 10. Shoot note
    // 11. Drive to far note
    // 12. Intake note
    // 13. Drive to shooting line
    // 14. Shoot note
    // 15. Drive to far note
    // 16. Intake note
    // 17. Drive to shooting line
    // 18. Shoot note
    
}

std::shared_ptr<Behaviour> QuadrupleCloseSingleFar(wom::drivetrain::SwerveDrive _driveBase, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return make<ArmToSetPoint>(_arm, 60)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoIntake>(_intake, 12)
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoIntake>(_intake, 12) 
    << make<ArmToSetPoint>(_arm, 0)
    << make<DriveToLocation>(_driveBase, 12, distance, direction)
    << make<ArmToSetPoint>(_arm, 180)
    << make<AutoShoot>(_shooter, 12)
    << make<ArmToSetPoint>(_arm, 0)

    
    // 4N Close 1N Far
    // 1. Shoot note
    // 2. Drive to close note
    // 3. Intake note
    // 4. Drive to speaker
    // 5. Shoot note
    // 6. Drive to close note
    // 7. Intake note
    // 8. Drive to speaker
    // 9. Shoot note
    // 10. Drive to close note
    // 11. Intake note
    // 12. Drive to speaker
    // 13. Shoot note
    // 14. Drive to far note
    // 15. Intake note
    // 15. Drive to speaker
    // 16. shoot 
    
}
   