// files should be named in upper camel case
// all .h or .hpp files should start with this as it causes the file to only compile its contents once at compile time
#pragma once

// your includes should be structured as shown below in your .h and .hpp files to increase readability
// Local includes, this means includes from the project you are working on eg 2023-ChargedUp

// Wombat includes

#include "behaviour/HasBehaviour.h"
#include "Gearbox.h"

// WPILIB includes

#include <frc/XboxController.h>

// Units includes

#include <units/time.h>

// CTRE includes

// REVLIB includes

// NAVX includes

// Standard library includes

#include <iostream>

// structs should use upper camel case in naming
// the first step in creating a subsystem is creating a configuration struct for it
// this struct should have all of the relevant motors, sensors, encoders and other required options for the code to work
// in this example the only config is a gearbox but configs can get much larger and more complicted

struct ExampleConfig {
  // all variables should be named using lower camel case

  wom::Gearbox leftGearbox;
};

// enums should use upper camel case naming
// all of the states for your subsystem should be encapsulated using an enum
// all subsystems should have a kIdle state for when the subsystem is not running

enum class ExampleState {
  // states should be named using a lowercase k then the state name in upper camel case

  kIdle,
  kRunning
};

// classes should be named in upper camel case
// all subsystems need a class which inherits from the has behaviour class

class Example: public behaviour::HasBehaviour {
 // the public field has data which can be accessed from any instance of the class

 public:
  // all classes require a constructor which tells you what you need to provide to create an instance of the class
  // constructors are declared as functions with no return type with the class name as the function name
  // constructors take in parameters, can assign those parameters to variables and also perform any logic a normal function can
  // all constructors should have a configuration variable passed in using a pointer as shown below and initialize that configuration to your 
  // _config variable as done is Example.cpp

  Example(ExampleConfig *_config, frc::XboxController &_driver);

  // all classes require a deconstructor which in most cases can just be empty
  // deconstructors are declared the same way as constructors but with a ~ at the front
  // the deconstructor tells the class what to do when the instance is deleted

  ~Example();

  // all functions are named in upper camel case and should be declared in the .h or .hpp file but given logic in the .cpp file
  // all subsystems require a function which will instruct it what to do when it starts named OnStart
  // in this function things like zeroing, logging or other tasks that are done on subsystem startup happen

  void OnStart();

  // all subsystems require a function which will instruct it what to do every tick (usually at 50 hertz but with a parameter to change frequency)
  // in this function you usually will have a switch statement for your states which will dictate what your states do along with other things that
  // happen during runtime eg logging

  void OnUpdate(units::second_t dt);

  // getters are functions which get a piece of data allowing it to be accessed with an instance of the class
  // all getters names should start with the word get
  // a common getter all subsystems should have is GetConfig which allows you to access the config with instances of the class

  ExampleConfig *GetConfig();

  // setters are functions which set a variables value based on input
  // all setters should start with the word set
  // a common example all subsystems should have is SetState which allows you to set the state of a subsystem

  void SetState(ExampleState state);

 // the protected field has data which can be accessed from within the class or within classes which inherit from it
 // it is usually not used but occasionally used for things like PID configs

 protected:

 // the private field has data which can be accessed only from within the class

 private:

 // all variables in the private field should start with an underscore
  // most subsystem private variables are config and state
  // these variables will store the configuration and state of the subsystem and are vital to its functioning

  ExampleConfig *_config;
  ExampleState _state;
  frc::XboxController &_driver;

};
