#include "utils/Gyro.h"

using namespace wom::utils;

#ifdef PLATFORM_ROBORIO
  #include <thread>
  #include "AHRS.h"
  class NavX::Impl {
   public:
    Impl() : ahrs(frc::SPI::kMXP) { }

    void Calibrate() {
      ahrs.Calibrate();
    }

    void Reset() {
      ahrs.Reset();
    }

    double GetAngle() const {
      return ahrs.GetAngle();
    }

    double GetRate() const {
      return ahrs.GetRate();
    }

    units::radian_t GetPitch() {
      return units::degree_t{ahrs.GetPitch()};
    }

    units::radian_t GetRoll() {
      return units::degree_t{ahrs.GetRoll()};
    }

    void SetAngle(units::radian_t offset) {
      Reset();
      this->offset = offset.convert<units::degree>().value();
    }
   private:
    AHRS ahrs;
    double offset;
  };
#else
  class NavX::Impl {
   public:
    void Calibrate() {}
    void Reset() { angle = 0; }
    double GetAngle() const { return angle; }
    // TODO:
    double GetRate() const { return 0; }

    units::radian_t GetPitch() { return 0_rad; }
    units::radian_t GetRoll() { return 0_rad; }

    void SetAngle(units::radian_t offset) {
      angle = offset.convert<units::degree>().value();
    }
   private:
    double angle{0};
  };
#endif

NavX::NavX() : impl(new NavX::Impl()) { }
NavX::~NavX() {
  delete impl;
}

void NavX::Calibrate() {
  impl->Calibrate();
}

void NavX::Reset() {
  impl->Reset();
}

double NavX::GetAngle() const {
  return impl->GetAngle();
}

double NavX::GetRate() const {
  return impl->GetRate();
}

units::radian_t NavX::GetPitch() {
  return impl->GetPitch();
}

units::radian_t NavX::GetRoll() {
  return impl->GetRoll();
}

void NavX::SetAngle(units::radian_t angle) {
  impl->SetAngle(angle);
}

