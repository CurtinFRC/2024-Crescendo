// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Auto.h"

#include <utility>

#include "AlphaArmBehaviour.h"
#include "IntakeBehaviour.h"
#include "ShooterBehaviour.h"
#include "behaviour/Behaviour.h"
#include "drivetrain/behaviours/SwerveBehaviours.h"
#include "units/angle.h"
#include "utils/Pathplanner.h"

using namespace wom;
using namespace behaviour;

// Shoots starting note then moves out of starting position.
wom::SwerveAutoBuilder* autos::InitCommands(wom::drivetrain::SwerveDrive* _swerveDrive, Shooter* _shooter,
                                            Intake* _intake, AlphaArm* _alphaArm) {
  wom::AutoCommands c = *new wom::AutoCommands(
      {// {"ArmToSetPoint", [_alphaArm]() { return wom::make<ArmToSetPoint>(_alphaArm, 20_deg); }},
       // {"Shoot", [_shooter]() { return wom::make<AutoShoot>(_shooter); }},
       {"IntakeNote", [_intake]() { return wom::make<IntakeNote>(_intake)->WithTimeout(1_s); }},
       {"PassNote", [_intake]() { return wom::make<PassNote>(_intake)->WithTimeout(1_s); }},
       {"EjectNote", [_intake]() { return wom::make<EjectNote>(_intake)->WithTimeout(1_s); }},
       {"Shoot", [_shooter, _intake]() { return wom::make<AutoShooter>(_shooter, _intake, 1500_rad_per_s); }} 
      });

  return new wom::utils::SwerveAutoBuilder(_swerveDrive, "Taxi", c);
}

std::shared_ptr<behaviour::Behaviour> autos::Taxi(wom::SwerveAutoBuilder* builder) {
  return builder->GetAutoRoutine("Taxi");
  // return behaviour::make<ArmToSetPoint>(_alphaArm, 0_deg);
  // behaviour::make<AutoShoot>(_shooter);
  // behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
}

std::shared_ptr<behaviour::Behaviour> autos::OneNoteTaxi(wom::SwerveAutoBuilder* builder) {
  return builder->GetAutoRoutine("OneNoteTaxi");
}

std::shared_ptr<behaviour::Behaviour> autos::ManualTaxi(wom::drivetrain::SwerveDrive* _swerveDrive, Shooter *_shooter, Intake *_intake, AlphaArm *_arm) {
  return 
  //WORKING PATH
    make<AutoShooter>(_shooter, _intake, 800_rad_per_s)->Until(make<WaitTime>(4.1_s))
    << make<PassNote>(_intake)->Until(make<WaitTime>(1_s))
    << make<ResetDrivebasePose>(_swerveDrive, true)
    << make<WaitTime>(0.1_s)
    << make<ResetDrivebasePose>(_swerveDrive, true)
    << make<IntakeNote>(_intake)->Until(make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{(_swerveDrive->GetPose().X() - 1.7_m), 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(1.5_s)))
    << make<WaitTime>(0.3_s)
    << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(1.5_s))
    << make<AutoShooter>(_shooter, _intake, 800_rad_per_s)->Until(make<WaitTime>(4_s))
    << make<PassNote>(_intake)->Until(make<WaitTime>(3_s));
    // << make<WaitTime>(1_s);

  //TESTING
    // make<ResetDrivebasePose>(_swerveDrive, true)
    // make<TurnToAngleBeh>(_swerveDrive, 180_deg)->Until(make<WaitTime>(5_s));

    // make<ResetDrivebasePose>(_swerveDrive, true)
    // << make<WaitTime>(1_s)
    // << make<ResetDrivebasePose>(_swerveDrive, true)
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{-0.01_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(1_s))
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{-1_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(4_s))
    // << make<WaitTime>(1_s)
    // // << make<ResetDrivebasePose>(_swerveDrive, true)
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{0.01_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(1_s))
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{1_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(4_s));
    // << make<ResetDrivebasePose>(_swerveDrive, true)
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{0.01_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(1_s))
    // << make<WaitTime>(1_s)
    // << make<ResetDrivebasePose>(_swerveDrive, true)
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{1_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(4_s))
    // << make<WaitTime>(1_s);

    // << make<TurnToAngleBeh>(_swerveDrive, 0_rad)->Until(make<WaitTime>(0.2_s))
    // << make<ResetDrivebasePose>(_swerveDrive, true)
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{0.01_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(0.2_s))
    // << make<WaitTime>(1_s)
    // << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{2_m, 0_m, 0_deg}, 6_V, true)->Until(make<WaitTime>(3_s))
    // << make<WaitTime>(1_s)
    // << make<TurnToAngleBeh>(_swerveDrive, 0_rad)->Until(make<WaitTime>(0.2_s));



    // make<AutoShooter>(_shooter, _intake, 800_rad_per_s)->Until(make<WaitTime>(4.1_s))
    //   << make<PassNote>(_intake)->Until(make<WaitTime>(1_s))
    //   << make<ResetDrivebasePose>(_swerveDrive, true)
    //   << make<TurnToAngleBeh>(_swerveDrive, 0_rad)->Until(make<WaitTime>(0.1_s))
    //   << make<ResetDrivebasePose>(_swerveDrive, true)
    //   << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{-0.01_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(0.2_s))
    //   << make<IntakeNote>(_intake)->Until(make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{-2_m, 0_m, 0_deg}, 6_V, true))
    //   << make<ResetDrivebasePose>(_swerveDrive, true)
    //   << make<TurnToAngleBeh>(_swerveDrive, 0_rad)->Until(make<WaitTime>(0.2_s))
    //   << make<ResetDrivebasePose>(_swerveDrive, true)
    //   << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{0.01_m, 0_m, 0_deg}, 3_V, true)->Until(make<WaitTime>(0.1_s))
    //   << make<DrivebasePoseBehaviour>(_swerveDrive, frc::Pose2d{1.2_m, 0_m, 0_deg}, 6_V, true)->Until(make<WaitTime>(3_s))
    //   << make<ResetDrivebasePose>(_swerveDrive, true)
    //   << make<TurnToAngleBeh>(_swerveDrive, 0_rad)->Until(make<WaitTime>(1_s))
    //   << make<AutoShooter>(_shooter, _intake, 800_rad_per_s)->Until(make<WaitTime>(1_s))
    //   << make<PassNote>(_intake)->Until(make<WaitTime>(1_s))
    //   << make<ResetDrivebasePose>(_swerveDrive, true)
    //   << make<TurnToAngleBeh>(_swerveDrive, 60_deg)->Until(make<WaitTime>(1_s));
}

// std::shared_ptr<behaviour::Behaviour> autos::QuadrupleClose(wom::drivetrain::SwerveDrive* _swerveDrive,
//                                                             Shooter* _shooter, Intake* _intake,
//                                                             AlphaArm* _alphaArm) {
//   return behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//
//   /*
//    4N Close
//  1. Shoot starting note into speaker
//  2. Intake note from close note
//  3. Shoot note into speaker
//  4. Intake note from close floor note
//  5. Shoot note into speaker
//  6. Intake not from close floor
//  7. Shoot note
//  */
// }
//
// std::shared_ptr<behaviour::Behaviour> autos::QuadrupleFar(wom::drivetrain::SwerveDrive* _swerveDrive,
//                                                           Shooter* _shooter, Intake* _intake,
//                                                           AlphaArm* _alphaArm) {
//   return behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//
//   /*
//     4N Far
//   1. Shoot start note in speaker
//   2. Drive to far note
//   3. Intake note
//   4. Drive back to shooting line
//   5. Shoot note into speaker
//   6. Drive to note
//   7. Intake note
//   8. Drive to shooting line
//   9. Shoot note
//   10. Drive to note
//   11. Intake note
//   12. Drive to shooting line
//   13. Shoot note
//   14. Drive to intake note (if possible)
//   */
// }
//
// std::shared_ptr<behaviour::Behaviour> autos::QuadrupleCloseDoubleFar(
//     wom::drivetrain::SwerveDrive* _swerveDrive, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm) {
//   return behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//
//   //   4N Close 2N Far
//   // 1. Shoot note
//   // 2. Drive to close note
//   // 3. Intake note
//   // 4. Shoot note
//   // 5. Drive to close note
//   // 6. Intake note
//   // 7. Shoot note
//   // 8. Drive to close note
//   // 9. Intake note
//   // 10. Shoot note
//   // 11. Drive to far note
//   // 12. Intake note
//   // 13. Drive to shooting line
//   // 14. Shoot note
//   // 15. Drive to far note
//   // 16. Intake note
//   // 17. Drive to shooting line
//   // 18. Shoot note
// }
//
// std::shared_ptr<behaviour::Behaviour> autos::QuadrupleCloseSingleFar(
//     wom::drivetrain::SwerveDrive* _swerveDrive, Shooter* _shooter, Intake* _intake, AlphaArm* _alphaArm) {
//   return behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoIntake>(_intake);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
// }
//
// // 4N Close 1N Far
// // 1. Shoot note
// // 2. Drive to close note
// // 3. Intake note
// // 4. Drive to speaker
// // 5. Shoot note
// // 6. Drive to close note
// // 7. Intake note
// // 8. Drive to speaker
// // 9. Shoot note
// // 10. Drive to close note
// // 11. Intake note
// // 12. Drive to speaker
// // 13. Shoot note
// // 14. Drive to far note
// // 15. Intake note
// // 15. Drive to speaker
// // 16. shoot
//
// //     /*
// //     TRAP AUTO
// //     1. Drive to trap
// //     2. Climb up
// //     3. Shoot note
// //     4. Climb down
// //     5. Drive to far note
// //     6. Intake note
// //     7. Drive to trap
// //     8. Climb up
// //     9. Shoot note
// //     10. Climb down
// //     11. Drive to far note
// //     12. Intake
// //     13. Drive to trap
// //     14. Climb up
// //     15. Shoot note
// //     16. Climb down
// //     */
// // }
//
// std::shared_ptr<behaviour::Behaviour> autos::AutoTest(wom::drivetrain::SwerveDrive* _swerveDrive,
//                                                       Shooter* _shooter, Intake* _intake,
//                                                       AlphaArm* _alphaArm) {
//   return behaviour::make<ArmToSetPoint>(_alphaArm, 1_deg);
//   behaviour::make<wom::drivetrain::behaviours::DrivebasePoseBehaviour>(
//       _swerveDrive, frc::Pose2d{0_m, 0_m, 0_deg}, 0_V, false);
//   behaviour::make<AutoShoot>(_shooter);
//   behaviour::make<AutoIntake>(_intake);
// }  // This auto is a test for auto to see if all things work.
