// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <units/time.h>

#include <functional>
#include <string>
#include <utility>

#include "behaviour/Behaviour.h"

namespace behaviour {

/**
 * @brief A class which binds behaviours to boolean conditions. Most commonly used through a wrapper around
 * a controller.
 *
 * Trigger is an API within the Behaviour scheduling to run different Behaviours based off boolean conditions.
 * It is instantiated with a condition that is evaluated on every Tick of the BehaviourScheduler causing the
 * OnTick method to run, checking the condition and running the relevant behaviour. Using the Trigger API
 * provides a variety of advantages to passing controllers into functions. By creating a Trigger using each
 * controller as a trigger based controller, we can easily figure out the control scheme of the entire robot.
 * We are also prevented from keybindings scheduling more than one Behaviour using the trigger controller
 * wrappers.
 */
class Trigger {
 public:
  /**
   * Creates a new Trigger that always returns false.
   *
   * @param name The name of the trigger, useful in debugging.
   */
  explicit Trigger(std::string name);

  /**
   * Creates a new Trigger that returns a boolean based off the provided condition.
   *
   * @param condition The condition to determine what to return. Evaluated every Tick.
   * @param name The name of the trigger, useful in debugging.
   */
  Trigger(std::function<bool()> condition, std::string name);

  /**
   * Creates a new Trigger that returns a boolean based off the provided condition.
   *
   * @param condition The condition to determine what to return. Evaluated every Tick.
   * @param name The name of the trigger, useful in debugging.
   * @param true_behaviour The behaviour to run when the condition is true.
   */
  Trigger(std::function<bool()> condition, std::string name, Behaviour* true_behaviour);

  /**
   * Creates a new Trigger that returns a boolean based off the provided condition.
   *
   * @param condition The condition to determine what to return. Evaluated every Tick.
   * @param name The name of the trigger, useful in debugging.
   * @param true_behaviour The behaviour to run when the condition is true.
   * @param false_behaviour The behaviour to run when the condition is false.
   */
  Trigger(std::function<bool()> condition, std::string name, Behaviour* true_behaviour,
          Behaviour* false_behaviour);

  ~Trigger() = default;
  Trigger(Trigger&&) = default;
  Trigger& operator=(Trigger&&) = default;

  /**
   * Returns the trigger's name.
   */
  std::string GetName();

  /**
   * A method that is run every tick. Checks whether or not the condition is true or false and runs the
   * associated behaviour.
   */
  void OnTick();

  /**
   * Sets the true behaviour to the provided behaviour.
   *
   * @param behaviour The behaviour to run on true.
   */
  void SetTrueBehaviour(Behaviour::ptr behaviour);

  /**
   * Sets the false behaviour to the provided behaviour.
   *
   * @param behaviour The behaviour to run on false.
   */
  void SetFalseBehaviour(Behaviour::ptr behaviour);

  /**
   * Returns a Trigger* which will be true when m_condition is not.
   */
  Trigger* operator!();

  /**
   * Returns a Trigger* from a logical OR with m_condition and the provided condition.
   *
   * @param rhs A std::pair of the right hand condition of the logical OR operation and the name for the new trigger.
   */
  Trigger* operator||(std::pair<std::function<bool()>, std::string> rhs);

  /**
   * Returns a Trigger* from a logical AND with m_condition and the provided condition.
   *
   * @param rhs A std::pair of the right hand condition of the logical AND operation and the name for the new trigger.
   */
  Trigger* operator&&(std::pair<std::function<bool()>, std::string> rhs);

 private:
  std::string k_name;
  std::function<bool()> m_condition;
  Behaviour::ptr m_true_behaviour = nullptr;
  Behaviour::ptr m_false_behaviour = nullptr;
};

}  // namespace behaviour
