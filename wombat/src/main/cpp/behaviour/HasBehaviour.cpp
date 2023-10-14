#include "behaviour/HasBehaviour.h"

#include "behaviour/Behaviour.h"

using namespace behaviour;

void HasBehaviour::SetDefaultBehaviour(
    std::function<std::shared_ptr<Behaviour>(void)> fn) {
  _default_behaviour_producer = fn;
}

std::shared_ptr<Behaviour> HasBehaviour::GetActiveBehaviour() {
  return _active_behaviour;
}