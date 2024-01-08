// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/RobotController.h>
#include <units/time.h>
#include <wpi/SmallSet.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "HasBehaviour.h"

namespace behaviour {
enum class BehaviourState { INITIALISED, RUNNING, DONE, TIMED_OUT, INTERRUPTED };

class SequentialBehaviour;

/**
 * A Behaviour is a single component in a chain of actions. Behaviours are used
 * to implement robot functionality and may include everything from running the
 * intake to following trajectories and scoring game pieces.
 *
 * Behaviours are ideally small components that are built into a much larger
 * chain, allowing each element to be tested and developed individually.
 *
 * For examples, see the BEHAVIOURS.md document.
 */
class Behaviour : public std::enable_shared_from_this<Behaviour> {
 public:
  using ptr = std::shared_ptr<Behaviour>;

  explicit Behaviour(std::string name = "<unnamed behaviour>", units::time::second_t period = 20_ms);
  ~Behaviour();

  /**
   * @return std::string The name of the Behaviour
   */
  virtual std::string GetName() const;

  /**
   * Called when the Behaviour first starts
   */
  virtual void OnStart() {}

  /**
   * Called periodically as the Behaviour runs
   * @param dt The time difference between the current call
   * and the previous call of OnTick
   */
  virtual void OnTick(units::time::second_t dt) = 0;

  /**
   * Called when the Behaviour stops running
   */
  virtual void OnStop() {}

  /**
   * Set the period of the Behaviour. Note this only affects the Behaviour
   * when scheduled using the BehaviourScheduler, or when used in as part
   * of a concurrent behaviour group (& or |)
   */
  void SetPeriod(units::time::second_t period);

  /**
   * @return units::time::second_t The loop period of the Behaviour.
   */
  units::time::second_t GetPeriod() const;

  /**
   * @return units::time::second_t The amount of time the Behaviour has been
   * running for.
   */
  units::time::second_t GetRunTime() const;

  /**
   * Specify what systems this Behaviour Controls. Controls means a physical
   * output, a demand, or some other controlling method. When Behaviours run,
   * only one Behaviour at a time may have control over a system.
   */
  void Controls(HasBehaviour *sys);

  /**
   * Inherit controlled systems from another Behaviour.
   */
  void Inherit(Behaviour &bhvr);

  /**
   * Set a timeout on this Behaviour. If the Behaviour runs longer than the
   * timeout, it will be interrupted.
   */
  ptr WithTimeout(units::time::second_t timeout);

  /**
   * @return wpi::SmallPtrSetImpl<HasBehaviour *>& The systems controlled by
   * this behaviour.
   */
  wpi::SmallPtrSetImpl<HasBehaviour *> &GetControlled();

  /**
   * @return BehaviourState The current state of the behaviour
   * @see BehaviourState
   */
  BehaviourState GetBehaviourState() const;

  /**
   * Interrupt this behaviour
   */
  void Interrupt();

  /**
   * Set this behaviour as being complete
   */
  void SetDone();

  /**
   * Tick this behaviour manually. It is very rare that you need to call this
   * function, as it will usually be done automatically by the
   * BehaviourScheduler. The only exception is when calling a behaviour within
   * another behaviour.
   */
  bool Tick();

  /**
   * Is this behaviour still running?
   */
  bool IsRunning() const;

  /**
   * Is this behaviour finished?
   */
  bool IsFinished() const;

  /**
   * Run this behaviour until another has finished.
   */
  Behaviour::ptr Until(Behaviour::ptr other);

 private:
  void Stop(BehaviourState new_state);

  std::string                 _bhvr_name;
  units::time::second_t       _bhvr_period = 20_ms;
  std::atomic<BehaviourState> _bhvr_state;

  wpi::SmallSet<HasBehaviour *, 8> _bhvr_controls;

  double                _bhvr_time    = 0;
  units::time::second_t _bhvr_timer   = 0_s;
  units::time::second_t _bhvr_timeout = -1_s;
};

/**
 * Shorthand function to create a shared_ptr for a Behaviour.
 */
template <class T, class... Args>
std::shared_ptr<T> make(Args &&...args) {
  return std::make_shared<T>(std::forward<Args>(args)...);
}

/**
 * The SequentialBehaviour runs a number of behaviours back-to-back, to
 * create a sequential chain of execution. Usually, you don't
 * want to invoke this class directly, but instead use b1 << b2.
 */
class SequentialBehaviour : public Behaviour {
 public:
  void Add(ptr next);

  std::string GetName() const override;

  void OnTick(units::time::second_t dt) override;
  void OnStop() override;

 protected:
  std::deque<ptr> _queue;
};

inline std::shared_ptr<SequentialBehaviour> operator<<(Behaviour::ptr a, Behaviour::ptr b) {
  auto seq = std::make_shared<SequentialBehaviour>();
  seq->Add(a);
  seq->Add(b);
  return seq;
}

inline std::shared_ptr<SequentialBehaviour> operator<<(std::shared_ptr<SequentialBehaviour> a,
                                                       Behaviour::ptr                       b) {
  a->Add(b);
  return a;
}

class DuplicateControlException : public std::exception {
 public:
  explicit DuplicateControlException(const std::string &msg) : _msg(msg) {}
  const char *what() const noexcept override { return _msg.c_str(); }

 private:
  std::string _msg;
};

enum class ConcurrentBehaviourReducer { ALL, ANY, FIRST };

/**
 * Create a concurrent set of behaviours that will run together.
 * Usually, you don't want to call this directly, but instead use b1 & b2 or b1
 * | b2 to create a concurrent group.
 */
class ConcurrentBehaviour : public Behaviour {
 public:
  explicit ConcurrentBehaviour(ConcurrentBehaviourReducer reducer);

  void Add(Behaviour::ptr behaviour);

  std::string GetName() const override;

  void OnStart() override;
  void OnTick(units::time::second_t dt) override;
  void OnStop() override;

 private:
  ConcurrentBehaviourReducer              _reducer;
  std::vector<std::shared_ptr<Behaviour>> _children;
  std::mutex                              _children_finished_mtx;
  std::vector<bool>                       _children_finished;
  std::vector<std::thread>                _threads;
};

/**
 * Create a concurrent behaviour group, waiting for all behaviours
 * to finish before moving on.
 */
inline std::shared_ptr<ConcurrentBehaviour> operator&(Behaviour::ptr a, Behaviour::ptr b) {
  auto conc = std::make_shared<ConcurrentBehaviour>(ConcurrentBehaviourReducer::ALL);
  conc->Add(a);
  conc->Add(b);
  return conc;
}

/**
 * Create a concurrent behaviour group, where unfinished behaviours will
 * be interrupted as soon as any members of the group are finished (the
 * behaviours are 'raced' against each other).
 */
inline std::shared_ptr<ConcurrentBehaviour> operator|(Behaviour::ptr a, Behaviour::ptr b) {
  auto conc = std::make_shared<ConcurrentBehaviour>(ConcurrentBehaviourReducer::ANY);
  conc->Add(a);
  conc->Add(b);
  return conc;
}

/**
 * If allows decisions to be made in a behaviour chain.
 */
struct If : public Behaviour {
 public:
  /**
   * Create a new If decision behaviour.
   * @param condition The condition to check, called when the behaviour is
   * scheduled.
   */
  explicit If(std::function<bool()> condition);
  /**
   * Create a new If decision behaviour
   * @param v The condition to check
   */
  explicit If(bool v);

  /**
   * Set the behaviour to be called if the condition is true
   */
  std::shared_ptr<If> Then(Behaviour::ptr b);

  /**
   * Set the behaviour to be called if the condition is false
   */
  std::shared_ptr<If> Else(Behaviour::ptr b);

  void OnStart() override;
  void OnTick(units::time::second_t dt) override;

 private:
  std::function<bool()> _condition;
  bool                  _value;
  Behaviour::ptr        _then, _else;
};

/**
 * The Switch behaviour is used to select from one of multiple paths,
 * depending on the value of a parameter.
 *
 * @tparam T The type of parameter.
 */
template <typename T = std::monostate>
struct Switch : public Behaviour {
 public:
  /**
   * Create a new Switch behaviour, with a given parameter
   * @param fn The function yielding the parameter, called in OnTick
   */
  explicit Switch(std::function<T()> fn) : _fn(fn) {}
  /**
   * Create a new Switch behaviour, with a given parameter
   * @param v The parameter on which decisions are made
   */
  explicit Switch(T v) : Switch([v]() { return v; }) {}

  /**
   * Add a new option to the Switch chain.
   *
   * @param condition The function yielding true if this is the correct option
   * @param b The behaviour to call if this option is provided.
   */
  std::shared_ptr<Switch> When(std::function<bool(T &)> condition, Behaviour::ptr b) {
    _options.push_back(std::pair(condition, b));
    Inherit(*b);
    return std::reinterpret_pointer_cast<Switch<T>>(shared_from_this());
  }

  /**
   * Add a new option to the Switch chain.
   *
   * @param value The option value, to be checked against the parameter
   * @param b The behaviour to call if this option is provided.
   */
  std::shared_ptr<Switch> When(T value, Behaviour::ptr b) {
    return When([value](T &v) { return value == v; }, b);
  }

  /**
   * Set the default branch for the Switch chain
   * @param b The behaviour to call if no other When's match.
   */
  std::shared_ptr<Switch> Otherwise(Behaviour::ptr b = nullptr) {
    return When([](T &v) { return true; }, b);
  }

  void OnTick(units::time::second_t dt) override {
    T val = _fn();

    if (!_locked) {
      for (auto &opt : _options) {
        if (opt.first(val)) {
          _locked = opt.second;

          if (_locked == nullptr) SetDone();
        }
      }
    }

    if (_locked) {
      SetPeriod(_locked->GetPeriod());
      if (_locked->Tick()) {
        SetDone();
      }
    }
  }

  void OnStop() override {
    if (GetBehaviourState() != BehaviourState::DONE) {
      for (auto &opt : _options) {
        opt.second->Interrupt();
      }
    }
  }

 private:
  std::function<T()>                                                       _fn;
  wpi::SmallVector<std::pair<std::function<bool(T &)>, Behaviour::ptr>, 4> _options;
  Behaviour::ptr                                                           _locked = nullptr;
};

/**
 * The Decide behaviour is a special case of Switch, where no parameter is
 * provided and is instead based purely on predicates.
 */
struct Decide : public Switch<std::monostate> {
  Decide() : Switch(std::monostate{}) {}

  /**
   * Add a new option to the Switch chain.
   *
   * @param condition The function yielding true if this is the correct option
   * @param b The behaviour to call if this option is provided.
   */
  std::shared_ptr<Decide> When(std::function<bool()> condition, Behaviour::ptr b) {
    return std::reinterpret_pointer_cast<Decide>(Switch::When([condition](auto) { return condition(); }, b));
  }
};

/**
 * The WaitFor behaviour will do nothing until a condition is true.
 */
struct WaitFor : public Behaviour {
 public:
  /**
   * Create a new WaitFor behaviour
   * @param predicate The condition predicate
   */
  explicit WaitFor(std::function<bool()> predicate);

  void OnTick(units::time::second_t dt) override;

 private:
  std::function<bool()> _predicate;
};

/**
 * The WaitTime behaviour will do nothing until a time period has elapsed.
 */
struct WaitTime : public Behaviour {
 public:
  /**
   * Create a new WaitTime behaviour
   * @param time The time period to wait
   */
  explicit WaitTime(units::time::second_t time);

  /**
   * Create a new WaitTime behaviour
   * @param time_fn The time period to wait, evaluated at OnStart
   */
  explicit WaitTime(std::function<units::time::second_t()> time_fn);

  void OnStart() override;
  void OnTick(units::time::second_t dt) override;

 private:
  std::function<units::time::second_t()> _time_fn;
  units::time::second_t                  _time;
};

struct Print : public Behaviour {
 public:
  explicit Print(std::string message);

  void OnTick(units::time::second_t dt) override;

 private:
  std::string _message;
};
}  // namespace behaviour
