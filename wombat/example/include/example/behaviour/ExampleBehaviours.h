#pragma once

#include "example/Example.h"

#include "behaviour/Behaviour.h"

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