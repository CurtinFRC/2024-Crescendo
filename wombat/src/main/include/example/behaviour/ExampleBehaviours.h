// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "behaviour/Behaviour.h"
#include "example/Example.h"

class ExampleBehaviour : public behaviour::Behaviour {
 public:
  explicit ExampleBehaviour(Example *_example);
  ~ExampleBehaviour();

  // See Example.h for more information about OnStart functions
  void OnStart();
  // See Example.h for more information about OnTick functions (use OnUpdate)
  void OnTick(units::second_t dt);

 private:
  Example *_example;
};
