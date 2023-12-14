// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/system/plant/DCMotor.h>

#include "VoltageController.h"
#include "utils/Encoder.h"

namespace wom {
namespace utils {
  /**
   * Struct for motor and encoder pairs.
   *
   * Combines the motor and encoder parts into one data structure,
   * so that both Spark + Encoder and Talon SRX are treated the same.
   */
  struct Gearbox {
    /**
     * The VoltageController (Motor Controller). May not be null.
     */
    VoltageController *transmission;

    /**
     * The Encoder. May be null, depending on the consumer of this structure.
     */
    Encoder *encoder = nullptr;

    /**
     * The motor being used. By default, this is a single NEO.
     */
    frc::DCMotor motor = frc::DCMotor::NEO(1);
  };
}  // namespace utils
}  // namespace wom
