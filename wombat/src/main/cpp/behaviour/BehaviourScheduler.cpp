#include "behaviour/BehaviourScheduler.h"

using namespace behaviour;

BehaviourScheduler::BehaviourScheduler() {}

BehaviourScheduler::~BehaviourScheduler() {
  for (HasBehaviour *sys : _systems) {
    if (sys->_active_behaviour) sys->_active_behaviour->Interrupt();
  }

  for (auto &t : _threads) {
    t.join();
  }
}

BehaviourScheduler *_scheduler_instance;

BehaviourScheduler *BehaviourScheduler::GetInstance() {
  if (_scheduler_instance == nullptr)
    _scheduler_instance = new BehaviourScheduler();
  return _scheduler_instance;
}

void BehaviourScheduler::Register(HasBehaviour *system) {
  _systems.push_back(system);
}

void BehaviourScheduler::Schedule(Behaviour::ptr behaviour) {
  if (behaviour->GetBehaviourState() != BehaviourState::INITIALISED) {
    throw std::invalid_argument("Cannot reuse Behaviours!");
  }

  std::lock_guard<std::recursive_mutex> lk(_active_mtx);

  for (HasBehaviour *sys : behaviour->GetControlled()) {
    if (sys->_active_behaviour != nullptr)
      sys->_active_behaviour->Interrupt();
    sys->_active_behaviour = behaviour;
  }

  _threads.emplace_back([behaviour, this]() {
    while (!behaviour->IsFinished()) {
      using namespace std::chrono_literals;

      {
        std::lock_guard<std::recursive_mutex> lk(_active_mtx);
        behaviour->Tick();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(
          (int64_t)(behaviour->GetPeriod().value() * 1000)));
    }
  });
}

void BehaviourScheduler::Tick() {
  std::lock_guard<std::recursive_mutex> lk(_active_mtx);
  for (HasBehaviour *sys : _systems) {
    if (sys->_active_behaviour != nullptr) {
      if (sys->_active_behaviour->IsFinished()) {
        if (sys->_default_behaviour_producer == nullptr) {
          sys->_active_behaviour = nullptr;
        } else {
          Schedule(sys->_default_behaviour_producer());
        }
      }
    } else if (sys->_default_behaviour_producer != nullptr) {
      Schedule(sys->_default_behaviour_producer());
    }
  }
}

void BehaviourScheduler::InterruptAll() {
  std::lock_guard<std::recursive_mutex> lk(_active_mtx);
  for (HasBehaviour *sys : _systems) {
    if (sys->_active_behaviour)
      sys->_active_behaviour->Interrupt();
  }
}