#include "Auto.h"

// std::shared_ptr<Behaviour> Taxi(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
//   return 
//     <<make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     <<make<AutoShoot>(_shooter, shooterVolt)
//     <<make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     <<make<DriveToLocation>(_driveBase, raw, distance)
//   //Shoots starting note then moves out of starting position.
// }

// std::shared_ptr<Behaviour> QuadrupleClose(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
//   return
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AuttoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoo>(_shooter, shooterVolt)

//     /* 
//       4N Close
//     1. Shoot starting note into speaker
//     2. Intake note from close note
//     3. Shoot note into speaker
//     4. Intake note from close floor note
//     5. Shoot note into speaker
//     6. Intake not from close floor
//     7. Shoot note
//     */
// }

// std::shared_ptr<Behaviour> QuadrupleFar(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
//   return
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     //<< make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     //<< make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     //<< make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     //<< make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     //<< make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     //<< make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     // << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     // << make<DriveToLocation>(_driveBase, raw, distance)   do this if possible
//     // << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     // << make<AutoIntake>(_intake, intakeVolt)

//     /*
//       4N Far
//     1. Shoot start note in speaker
//     2. Drive to far note
//     3. Intake note
//     4. Drive back to shooting line
//     5. Shoot note into speaker
//     6. Drive to note
//     7. Intake note
//     8. Drive to shooting line
//     9. Shoot note
//     10. Drive to note
//     11. Intake note
//     12. Drive to shooting line
//     13. Shoot note
//     14. Drive to intake note (if possible)
//     */
// }

// std::shared_ptr<Behaviour> QuadrupleCloseDoubleFar(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
//   return
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     //<< make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     //<< make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     //<< make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     //<< make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     //<< make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     //<< make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     /*
//       4N Close 2N Far
//     1. Shoot note
//     2. Drive to close note
//     3. Intake note
//     4. Shoot note
//     5. Drive to close note
//     6. Intake note
//     7. Shoot note
//     8. Drive to close note
//     9. Intake note
//     10. Shoot note
//     11. Drive to far note
//     12. Intake note
//     13. Drive to shooting line
//     14. Shoot note
//     15. Drive to far note
//     16. Intake note
//     17. Drive to shooting line
//     18. Shoot note
//     */
// }

// std::shared_ptr<Behaviour> QuadrupleCloseSingleFar(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
//   return
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)

//     /*
//     4N Close 1N Far
//     1. Shoot note
//     2. Drive to close note
//     3. Intake note
//     4. Drive to speaker
//     5. Shoot note
//     6. Drive to close note
//     7. Intake note
//     8. Drive to speaker
//     9. Shoot note
//     10. Drive to close note
//     11. Intake note
//     12. Drive to speaker
//     13. Shoot note
//     14. Drive to far note
//     15. Intake note
//     15. Drive to speaker
//     16. shoot 
//     */
// }

// std::shared_ptr<Behaviour> TrapAuto(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
//   return 
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<>
//     << make<ArmToSetPoint>(_arm, armAngle, wristAngle)
//     << make<AutoShoot>(_shooter, shooterVolt)
//     << make<>
//     << make<DriveToLocation>(_driveBase, raw, distance)
//     << make<AutoIntake>(_intake, intakeVolt)
//     << make<DriveToLocation>(_driveBase, raw, distance)
    
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

std::shared_ptr<Behaviour> AutoTest(DriveBase _driveBase, Shooter _shooter, Mag _mag, Intake _intake, Arm _arm) {
  return
    <<make<ArmToSetPoint>(_arm, 0, -90)
    <<make<DriveToLocation>(_driveBase, raw, distance)
    <<make<AutoShoot>(_shooter, 8_V)
    <<make<AutoIntake>(_intake, 8_V)
} // This auto is a test for auto to see if all things work, it does not build as the behaviours are not done.