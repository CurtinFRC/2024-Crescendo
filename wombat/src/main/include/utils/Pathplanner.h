#pragma once

#include <units/voltage.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/length.h>

namespace wom {
namespace utils {
    class Pathplanner {
     public:
      Pathplanner(double ks, double kv, double ka, double kp, double trackwidth)
          : _ks(ks), _kv(kv), _ka(ka), _kp(kp), _trackwidth(trackwidth) {}

      double GetKs() const { return _ks; }
      double GetKv() const { return _kv; }
      double GetKa() const { return _ka; }
      double GetPDriveVel() const { return _kPDriveVel; }
      double GetTrackWidth() const { return _kTrackwidth; }



     private:
      units::volt _ks;
      auto _kv;
      auto _ka;
      double _kPDriveVel;
      units::meter _kTrackwidth;

      units::meters_per_second_t _kMaxSpeed;
      units::meters_per_second_squared_t _kMaxAcceleration;

      auto kRamseteB;
      auto kRamseteZeta;

      frc::ADXRS450_Gyro m_gyro;
    };
}
}
// TODO: Tune the values to our drivebase

constexpr auto ks = 0.22_V;
constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

constexpr double kPDriveVel = 8.5;

constexpr auto kTrackwidth = 0.69_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;