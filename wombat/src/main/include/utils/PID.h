// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/filter/LinearFilter.h>
#include <networktables/NetworkTableInstance.h>
#include <units/base.h>
#include <units/time.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "utils/Util.h"

namespace wom {
namespace utils {
  template <typename IN, typename OUT>
  struct PIDConfig {
    using in_t = units::unit_t<IN>;

    using kp_t = units::unit_t<units::compound_unit<OUT, units::inverse<IN>>>;
    using ki_t = units::unit_t<units::compound_unit<OUT, units::inverse<IN>, units::inverse<units::second>>>;
    using kd_t = units::unit_t<units::compound_unit<OUT, units::inverse<IN>, units::second>>;

    using error_t = units::unit_t<IN>;
    using deriv_t = units::unit_t<units::compound_unit<IN, units::inverse<units::second>>>;

    PIDConfig(std::string path, kp_t kp = kp_t{0}, ki_t ki = ki_t{0}, kd_t kd = kd_t{0},
              error_t stableThresh = error_t{-1}, deriv_t stableDerivThresh = deriv_t{-1},
              in_t izone = in_t{-1})
        : path(path),
          kp(kp),
          ki(ki),
          kd(kd),
          stableThresh(stableThresh),
          stableDerivThresh(stableDerivThresh),
          izone(izone) {
      RegisterNT();
    }

    std::string path;

    kp_t kp;
    ki_t ki{0};
    kd_t kd{0};

    error_t stableThresh{-1};
    deriv_t stableDerivThresh{-1};

    in_t izone{-1};

   private:
    std::vector<std::shared_ptr<utils::NTBound>> _nt_bindings;

   public:
    void RegisterNT() {
      auto table = nt::NetworkTableInstance::GetDefault().GetTable(path);
      _nt_bindings.emplace_back(
          std::make_shared<utils::NTBoundUnit<typename kp_t::unit_type>>(table, "kP", kp));
      _nt_bindings.emplace_back(
          std::make_shared<utils::NTBoundUnit<typename ki_t::unit_type>>(table, "kI", ki));
      _nt_bindings.emplace_back(
          std::make_shared<utils::NTBoundUnit<typename kd_t::unit_type>>(table, "kD", kd));
      _nt_bindings.emplace_back(std::make_shared<utils::NTBoundUnit<typename error_t::unit_type>>(
          table, "stableThresh", stableThresh));
      _nt_bindings.emplace_back(std::make_shared<utils::NTBoundUnit<typename deriv_t::unit_type>>(
          table, "stableThreshVelocity", stableDerivThresh));
      _nt_bindings.emplace_back(std::make_shared<utils::NTBoundUnit<IN>>(table, "izone", izone));
    }
  };

  template <typename IN, typename OUT>
  class PIDController {
   public:
    using config_t = PIDConfig<IN, OUT>;
    using in_t     = units::unit_t<IN>;
    using out_t    = units::unit_t<OUT>;
    using sum_t    = units::unit_t<units::compound_unit<IN, units::second>>;

    config_t config;

    PIDController(std::string path, config_t initialGains, in_t setpoint = in_t{0})
        : config(initialGains),
          _setpoint(setpoint),
          _posFilter(frc::LinearFilter<typename config_t::error_t>::MovingAverage(20)),
          _velFilter(frc::LinearFilter<typename config_t::deriv_t>::MovingAverage(20)),
          _table(nt::NetworkTableInstance::GetDefault().GetTable(path)) {}

    void SetSetpoint(in_t setpoint) {
      if (std::abs(setpoint.value() - _setpoint.value()) > std::abs(0.1 * _setpoint.value())) {
        _iterations = 0;
      }
      _setpoint = setpoint;
    }

    in_t GetSetpoint() const { return _setpoint; }

    in_t GetError() const { return _last_error; }

    void SetWrap(std::optional<in_t> range) { _wrap_range = range; }

    void Reset() { _integralSum = sum_t{0}; }

    out_t Calculate(in_t pv, units::second_t dt, out_t feedforward = out_t{0}) {
      auto error = do_wrap(_setpoint - pv);
      _integralSum += error * dt;
      if (config.izone.value() > 0 && (error > config.izone || error < -config.izone))
        _integralSum = sum_t{0};

      typename config_t::deriv_t deriv{0};

      if (_iterations > 0) deriv = (pv - _last_pv) / dt;

      _stablePos = _posFilter.Calculate(error);
      _stableVel = _velFilter.Calculate(deriv);

      auto out = config.kp * error + config.ki * _integralSum + config.kd * deriv + feedforward;
      // std::cout << "Out value" << out.value() << std::endl;

      _table->GetEntry("pv").SetDouble(pv.value());
      _table->GetEntry("dt").SetDouble(dt.value());
      _table->GetEntry("setpoint").SetDouble(_setpoint.value());
      _table->GetEntry("error").SetDouble(error.value());
      _table->GetEntry("integralSum").SetDouble(_integralSum.value());
      _table->GetEntry("stable").SetBoolean(IsStable());
      _table->GetEntry("demand").SetDouble(out.value());

      _last_pv    = pv;
      _last_error = error;
      _iterations++;
      return out;
    }

    bool IsStable(std::optional<typename config_t::error_t> stableThreshOverride   = {},
                  std::optional<typename config_t::deriv_t> velocityThreshOverride = {}) const {
      auto stableThresh      = config.stableThresh;
      auto stableDerivThresh = config.stableDerivThresh;

      if (stableThreshOverride.has_value()) stableThresh = stableThreshOverride.value();
      if (velocityThreshOverride.has_value()) stableDerivThresh = velocityThreshOverride.value();

      return _iterations > 20 && std::abs(_stablePos.value()) <= std::abs(stableThresh.value()) &&
             (stableDerivThresh.value() < 0 || std::abs(_stableVel.value()) <= stableDerivThresh.value());
    }

   private:
    in_t do_wrap(in_t val) {
      if (_wrap_range.has_value()) {
        double wr = _wrap_range.value().value();
        double v  = val.value();

        v = std::fmod(v, wr);
        if (std::abs(v) > (wr / 2.0)) {
          return in_t{(v > 0) ? v - wr : v + wr};
        } else {
          return in_t{v};
        }
      }
      return val;
    }

    in_t  _setpoint;
    sum_t _integralSum{0};
    in_t  _last_pv{0}, _last_error{0};

    std::optional<in_t> _wrap_range;

    int _iterations = 0;

    frc::LinearFilter<typename config_t::error_t> _posFilter;
    frc::LinearFilter<typename config_t::deriv_t> _velFilter;

    typename config_t::error_t _stablePos;
    typename config_t::deriv_t _stableVel;

    std::shared_ptr<nt::NetworkTable> _table;
  };
}  // namespace utils
}  // namespace wom
