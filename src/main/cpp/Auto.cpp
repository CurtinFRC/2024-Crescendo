#include "Auto.h"

std::shared_ptr<behaviour::Behaviour> Taxi(wom::drivetrain::SwerveDrive _driveBase, frc::Timer* timer, frc::Field2d* field, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
}
// Shoots starting note then moves out of starting position.

std::shared_ptr<behaviour::Behaviour> QuadrupleClose(wom::drivetrain::SwerveDrive _driveBase, frc::Timer* timer, frc::Field2d* field, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);

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

std::shared_ptr<behaviour::Behaviour> QuadrupleFar(wom::drivetrain::SwerveDrive _driveBase, frc::Timer* timer, frc::Field2d* field, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);   // do this if possible
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);

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

std::shared_ptr<behaviour::Behaviour> QuadrupleCloseDoubleFar(wom::drivetrain::SwerveDrive _driveBase, frc::Timer* timer, frc::Field2d* field, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    
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

std::shared_ptr<behaviour::Behaviour> QuadrupleCloseSingleFar(wom::drivetrain::SwerveDrive _driveBase, frc::Timer* timer, frc::Field2d* field, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoIntake>(&_intake, 8_V); 
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
}
    
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
    
//     /* 
//     TRAP AUTO
//     1. Drive to trap
//     2. Climb up
//     3. Shoot note
//     4. Climb down
//     5. Drive to far note
//     6. Intake note
//     7. Drive to trap
//     8. Climb up
//     9. Shoot note
//     10. Climb down
//     11. Drive to far note
//     12. Intake
//     13. Drive to trap 
//     14. Climb up
//     15. Shoot note
//     16. Climb down
//     */
// }

std::shared_ptr<behaviour::Behaviour> autos::AutoTest(wom::drivetrain::SwerveDrive _driveBase, frc::Timer* timer, frc::Field2d* field, Shooter _shooter, Intake _intake, AlphaArm _alphaArm) {
  return behaviour::make<ArmToSetPoint>(&_alphaArm, 0 * 1_deg, 0.2);
    behaviour::make<wom::drivetrain::behaviours::AutoSwerveDrive>(&_driveBase, timer, field);
    behaviour::make<AutoShoot>(&_shooter, 8_V);
    behaviour::make<AutoIntake>(&_intake, 8_V);
} // This auto is a test for auto to see if all things work. 