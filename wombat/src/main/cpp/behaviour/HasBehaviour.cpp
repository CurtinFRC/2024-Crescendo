// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "behaviour/HasBehaviour.h"

#include "behaviour/Behaviour.h"

using namespace behaviour;

void HasBehaviour::SetDefaultBehaviour(std::function<std::shared_ptr<Behaviour>(void)> fn) {
  _default_behaviour_producer = fn;
}

std::shared_ptr<Behaviour> HasBehaviour::GetActiveBehaviour() {
  return _active_behaviour;
}
