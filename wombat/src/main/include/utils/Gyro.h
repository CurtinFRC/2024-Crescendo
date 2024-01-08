#pragma once

#include <frc/interfaces/Gyro.h>

namespace wom {
  namespace utils {
  class Gyro : public frc::Gyro {
   public:
  };

  class NavX : public Gyro {
   public:
    NavX();
    ~NavX();
    /* From frc::Gyro */
    void Calibrate() override;
    void Reset() override;
    double GetAngle() const override;
    double GetRate() const override;

    units::radian_t GetPitch();
    units::radian_t GetRoll();

    void SetAngle(units::radian_t angle);

   private:
    class Impl;
    Impl *impl;
  };
  }
}