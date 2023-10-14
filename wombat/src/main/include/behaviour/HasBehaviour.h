#pragma once

#include <functional>
#include <memory>

namespace behaviour {
class Behaviour;
class BehaviourScheduler;

/**
 * HasBehaviour is applied to a system that can be controlled by behaviours.
 * This is commonly implemented on shooters, drivetrains, elevators, etc.
 */
class HasBehaviour {
 public:
  /**
   * Set the default behaviour to run if no behaviours are currently running.
   * This is commonly used to default to Teleoperated control.
   */
  void SetDefaultBehaviour(std::function<std::shared_ptr<Behaviour>(void)> fn);

  /**
   * Get the currently running behaviour on this system.
   */
  std::shared_ptr<Behaviour> GetActiveBehaviour();

 protected:
  std::shared_ptr<Behaviour>                      _active_behaviour{nullptr};
  std::function<std::shared_ptr<Behaviour>(void)> _default_behaviour_producer{nullptr};

 private:
  friend class BehaviourScheduler;
};
}  // namespace behaviour