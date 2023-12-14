// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/RobotStartup.h"

void wom::utils::RobotStartup::Start(std::function<int()> robotFunc) {
  robotFunc();
}
    