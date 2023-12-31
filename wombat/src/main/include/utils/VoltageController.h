#pragma once

#include <frc/motorcontrol/MotorController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <units/voltage.h>

#include "utils/Util.h"

namespace wom {
namespace utils {
  /**
   * A VoltageController is analagous to a MotorController, but in terms of voltage instead
   * of speed.
   */
  class VoltageController {
   public:
    VoltageController(frc::MotorController *MotorController);
    /**
     * Set the voltage of the output.
     */
    void SetVoltage(units::volt_t voltage);
    /**
     * Get the voltage of the output.
     */
    units::volt_t GetVoltage() const;

    /**
     * Set the output as inverted.
     */
    void SetInverted(bool invert);
    /**
     * Get whether the output is inverted
     */
    bool GetInverted() const;

    /**
     * Get the estimated real voltage of the output, based on the controller voltage.
     */
    units::volt_t GetEstimatedRealVoltage() const;

    units::volt_t GetBusVoltage() const;

  /**
  * Create a MotorVoltageController with a given frc::MotorController
  * subclass. Please note that this creates an unsafe pointer (will never dealloc)
  */
  template<typename T, typename ...Args>
  static VoltageController Of(Args& ...args) {
    T *t = new T(args...);  // Be warned, does not deallocate!
    return VoltageController{t};
  }

  template<typename ...Args>
  static VoltageController Group(Args& ...args) {
   return Of<frc::MotorControllerGroup>(args...);
  }

   private:
    frc::MotorController *_MotorController;
  };
}
}  // namespace wom
