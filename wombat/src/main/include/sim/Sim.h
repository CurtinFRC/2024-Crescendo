#include "drivetrain/SwerveDrive.h"
#include "frc/smartdashboard/Field2d.h"
namespace wom {
namespace sim {
class SimSwerve {
public:
  SimSwerve(drivetrain::SwerveDrive* _swerve);

  void OnTick();

private:
  frc::Field2d _field;
  drivetrain::SwerveDrive* _swerve;
};
}
}
