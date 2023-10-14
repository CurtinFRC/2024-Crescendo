// This should be the only include in your subsystem files
#include "example/Example.h"

// here is the declaration for the example class's constructor
// first we have our function declaration again as done before
// then we have the initializer which is entered using a colon
// the initializer takes a variable which should then have brackets added to the end of it as if it was a function and then assigns that variable
// the value or the parameter placed inside the brackets
// we see this done here with the most common usage assigning _config the subsystem's configuration
// you can also add more logic as you would inside a normal function but that is not normally necessary

Example::Example(ExampleConfig *_config, frc::XboxController &_driver): _config(_config), _driver(_driver) {}

// here we have our set state function. inside of the logic all we need to do for a setter is take the variable we want and assign it the 
// parameters value so they are usually one line functions

void Example::SetState(ExampleState state) { _state = state; }

// here we have our get config. inside of the logic all we need to do is return our config variable making getters also quite simple and usually
// one liners

ExampleConfig *Example::GetConfig() { return _config; }

// here we have our on start function. all this on start function does is print starting to the console when the robot starts but you will often
// also have to add zeroing and other features

void Example::OnStart() {
  std::cout << "starting" << std::endl;
}

// here we have our on update function. this function is relatively simple in this example but in most subsystems will be much more complicated.
// all we are doing here switching between states based on _state and performing certain logic based on what state we are in. in kIdle we do nothing
// the point of the kIdle state. in kRunning we use a ternary operator to change the speed of our motor based on the y value of the left joystick
// on our controller

void Example::OnUpdate(units::second_t dt) {
  switch(_state) {
    case ExampleState::kIdle:
      break;
    case ExampleState::kRunning:
      double speed = (fabs(_driver.GetLeftY()) > 0.15) ? speed : 0;
      _config->leftGearbox.transmission->SetVoltage(speed * 1_V);
      break;
  }
}
