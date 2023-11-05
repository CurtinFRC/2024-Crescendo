#pragma once

#include "behaviour/Behaviour.h"
#include "example/Example.h"

class ExampleBehaviour : public behaviour::Behaviour {
public:
  ExampleBehaviour(Example *_example);
  ~ExampleBehaviour();

  // See Example.h for more information about OnStart functions
  void OnStart();
  // See Example.h for more information about OnTick functions (use OnUpdate)
  void OnTick(units::second_t dt);

private:
  Example *_example;
};
