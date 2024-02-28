#include "drivetrain/SwerveDrive.h"
#include "frc/geometry/Pose2d.h"
#include "frc/smartdashboard/Field2d.h"
namespace wom {
namespace sim {
class SimSwerve {
public:
  SimSwerve(drivetrain::SwerveDrive* _swerve);

  void OnTick();
  void OnTick(frc::Pose2d pose);

private:
  frc::Field2d _field;
  drivetrain::SwerveDrive* _swerve;
};
}
}
