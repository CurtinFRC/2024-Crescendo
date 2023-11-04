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
    /**
     * Set the voltage of the output.
     */
    virtual void SetVoltage(units::volt_t voltage) = 0;
    /**
     * Get the voltage of the output.
     */
    virtual units::volt_t GetVoltage() const = 0;

    /**
     * Set the output as inverted.
     */
    virtual void SetInverted(bool invert) = 0;
    /**
     * Get whether the output is inverted
     */
    virtual bool GetInverted() const = 0;

    /**
     * Get the estimated real voltage of the output, based on the controller voltage. 
     */
    units::volt_t GetEstimatedRealVoltage() const;
  };

  /**
   * The MotorVoltageController is an adapter for an frc::MotorController to
   * a VoltageController.
   */
  class MotorVoltageController : public VoltageController {
   public:
    MotorVoltageController(frc::MotorController *MotorController);

    void SetVoltage(units::volt_t voltage) override;
    units::volt_t GetVoltage() const override;

    void SetInverted(bool invert) override;
    bool GetInverted() const override;

    units::volt_t GetBusVoltage() const;

    /**
     * Create a MotorVoltageController with a given frc::MotorController
     * subclass. Please note that this creates an unsafe pointer (will never dealloc)
     */
    template<typename T, typename ...Args>
    static MotorVoltageController Of(Args& ...args) {
      T *t = new T(args...);  // Be warned, does not deallocate!
      return MotorVoltageController{t};
    }

    template<typename ...Args>
    static MotorVoltageController Group(Args& ...args) {
      return Of<frc::MotorControllerGroup>(args...);
    }

   private:
    frc::MotorController *_MotorController;
  };
} // namespace utils
}  // namespace wom
