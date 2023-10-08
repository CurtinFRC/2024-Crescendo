// this should be your only include in your behaviour cpp files
#include "example/behaviour/ExampleBehaviours.h"

ExampleBehaviour::ExampleBehaviour(Example *_example): _example(_example) {
  Controls(_example);
}

void ExampleBehaviour::OnStart() {
  _example->OnStart();
  std::cout << "behaviour starting" << std::endl;
}

void ExampleBehaviour::OnTick(units::second_t dt) {
  _example->SetState(ExampleState::kRunning);
  // _example->OnUpdate(dt);
}