#include "utils/RobotStartup.h"

void wom::utils::RobotStartup::Start(std::function<int()> robotFunc) {
  robotFunc();
}
