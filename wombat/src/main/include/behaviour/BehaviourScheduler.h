// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <mutex>
#include <vector>

#include "Behaviour.h"
#include "HasBehaviour.h"

namespace behaviour {

/**
 * The BehaviourScheduler is the primary entrypoint for running behaviours.
 * Behaviours are scheduled with Schedule(...), and systems are registered with
 * Register(...).
 *
 * The scheduler Tick() method must be called on a regular basis, such as in
 * RobotPeriodic
 */
class BehaviourScheduler {
 public:
  BehaviourScheduler();
  ~BehaviourScheduler();

  /**
   * @return BehaviourScheduler* The global instance of the BehaviourScheduler
   */
  static BehaviourScheduler *GetInstance();

  /**
   * Register a system with the behaviour scheduler. A system must be registered
   * for it to be controlled by behaviours.
   */
  void Register(HasBehaviour *system);

  /**
   * Schedule a behaviour, interrupting all behaviours currently running that
   * control the same system.
   */
  void Schedule(Behaviour::ptr behaviour);

  /**
   * Update the BehaviourScheduler. Must be called regularly, e.g. RobotPeriodic
   */
  void Tick();

  /**
   * Interrupt all running behaviours. This is commonly called on DisabledInit
   * and TeleopInit.
   */
  void InterruptAll();

 private:
  std::vector<HasBehaviour *> _systems;
  std::recursive_mutex        _active_mtx;
  std::vector<std::thread>    _threads;
};
}  // namespace behaviour
