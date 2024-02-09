// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "behaviour/Trigger.h"

#include <iostream>

#include "behaviour/BehaviourScheduler.h"

using namespace behaviour;

Trigger::Trigger(std::string name) : k_name{name}, m_condition{[]() { return false; }} {
  behaviour::BehaviourScheduler::GetInstance()->AddTrigger(this);
}

Trigger::Trigger(std::function<bool()> condition, std::string name) : k_name{name}, m_condition(condition) {
  behaviour::BehaviourScheduler::GetInstance()->AddTrigger(this);
}

Trigger::Trigger(std::function<bool()> condition, std::string name, Behaviour* true_behaviour)
    : k_name{name}, m_condition(condition), m_true_behaviour{true_behaviour} {
  behaviour::BehaviourScheduler::GetInstance()->AddTrigger(this);
}

Trigger::Trigger(std::function<bool()> condition, std::string name, Behaviour* true_behaviour,
                 Behaviour* false_behaviour)
    : k_name{name},
      m_condition(condition),
      m_true_behaviour{true_behaviour},
      m_false_behaviour{false_behaviour} {
  behaviour::BehaviourScheduler::GetInstance()->AddTrigger(this);
}

void Trigger::OnTick() {
  if (m_condition()) {
    if (m_true_behaviour != nullptr) {
      BehaviourScheduler::GetInstance()->Schedule(m_false_behaviour);
      return;
    }
    std::cerr << "WARNING: No OnTrue behaviour specified for Trigger " << k_name << std::endl;
  }
  if (m_false_behaviour != nullptr) {
    BehaviourScheduler::GetInstance()->Schedule(m_false_behaviour);
    return;
  }
  std::cerr << "WARNING: No OnFalse behaviour specified for Trigger " << k_name << std::endl;
}

void Trigger::SetTrueBehaviour(Behaviour::ptr behaviour) {
  m_true_behaviour = behaviour;
}

void Trigger::SetFalseBehaviour(Behaviour::ptr behaviour) {
  m_false_behaviour = behaviour;
}

Trigger* Trigger::operator!() {
  return new Trigger([condition = m_condition]() { return condition(); }, "Not" + k_name);
}

Trigger* Trigger::operator||(std::function<bool()> rhs) {
  return new Trigger([condition = m_condition, rhs = std::move(rhs)]() { return condition() || rhs(); },
                     "Logical or of " + k_name);
}

Trigger* Trigger::operator&&(std::function<bool()> rhs) {
  return new Trigger([condition = m_condition, rhs = std::move(rhs)]() { return condition() && rhs(); },
                     "Logical and of " + k_name);
}
