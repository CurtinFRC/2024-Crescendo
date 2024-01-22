#include "Auto.h"

std::shared_ptr<Behaviour> QuadrupleClose(/*Drivebase drivebase, Shooter shooter, Mag mag, Intake intake*/) {
  return
    << make<Shoot>()
    << make<Drive>()
    << make<Intake>()
    << make<Shoot>()
    << make<Drive>()
    << make<Intake>()
    << make<Shoot>()
    << make<Drive>()
    << make<Intake>()
    << make<Shoot>()
    /* 
    1. Shoot starting note into speaker
    2. Intake note from close note
    3. Shoot note into speaker
    4. Intake note from close floor note
    5. Shoot note into speaker
    6. Intake not from close floor
    7. Shoot note
    */
}

