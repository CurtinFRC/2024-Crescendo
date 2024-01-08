# Behaviours

Behaviours are Curtin FRC's solution to the control problem. In a robot where many actions can occur at once, how do we make sure only one place has control of a system at a particular time? Moreover, how can we combine small elements of control into one much larger, cohesive functionality - such as chaining small autonomous actions together to make a full autonomous routine?

A Behaviour is a small action or piece of work that describes how to control a robot system or collection of systems. Examples may include:
- Teleoperated control of the drivetrain
- Automatic spinup of the shooter
- Setting the elevator to go to a specific height
- Performing motor calibration
- ... and many more.

The key to Behaviours is that they are *small*, and are used together to build into a much larger system.

## Implementing Behaviours
Behaviours are created by inheriting from the `Behaviour` class. Let's look at an example Behaviour that prints a message when it starts, and then immediately stops.

```cpp
// PrintBehaviour.h
#pragma once
#include <behaviour/Behaviour.h>

class PrintBehaviour : public behaviour::Behaviour {
 public:
  PrintBehaviour(std::string message);

  // OnStart and OnStop are called as when the Behaviour is initially started,
  // then again when the behaviour is finished. Both of these overrides are
  // optional, but OnTick is mandatory.
  void OnStart() override;
  // OnTick is called periodically (over and over again) while the behaviour
  // is running. This is where most logic goes, and also where the Behaviour
  // decides whether it is done by calling SetDone().
  void OnTick(units::time::second_t dt) override;
  void OnStop() override;

 private:
  std::string _message;
};

// PrintBehaviour.cpp
#include "PrintBehaviour.h"

using namespace behaviour;

// Note the call to Behaviour(std::string), which is how you give a Behaviour
// a name that can be read out on Shuffleboard / in the console. You can
// also call it without a string to make an unnamed behaviour.
PrintBehaviour::PrintBehaviour(std::string message) : _message(message), Behaviour("PrintBehaviour(" + message + ")") {}

void PrintBehaviour::OnStart() {
  // When I start, I print my message
  std::cout << "Hello World! My message is: " << _message << std::endl;
}

void PrintBehaviour::OnTick() {
  // I have no periodic behaviour, so I say I'm done right away.
  SetDone();
}

void PrintBehaviour::OnStop() {
  // And I say goodbye :)
  std::cout << "I'm out of here, see ya!" << std::endl;
}
```
We can run this behaviour by doing the following:
```cpp
auto behaviour = make<PrintBehaviour>("Hello!");
BehaviourScheduler::GetInstance()->Schedule(behaviour);
```

### Controlling systems
Let's look at how we might control a system with Behaviours. For this example, we're going to create a Behaviour that tells a flywheel shooter to spin up to a certain speed.

First, we need to make our shooter system able to have a Behaviour. To do this, we make it implement from `HasBehaviour` - that's it, no methods to override or anything else.

```cpp
// Shooter.h

class Shooter : public HasBehaviour {
  // ...
};
```
Go ahead and register the Shooter with the BehaviourScheduler:
```cpp
void Robot::RobotInit() {
  // ...
  BehaviourScheduler::GetInstance()->Register(&my_shooter);
  // ...
}
```

Next, we create our Behaviour. Just for fun, let's do some PIDF while we're at it.

```cpp
// ShooterSpinup.h
#pragma once
#include <behaviour/Behaviour.h>
#include <PIDController.h>
#include <units/angular_velocity.h>

class ShooterSpinup : public behaviour::Behaviour {
 public:
  ShooterSpinup(Shooter &s, units::rad_per_s speed, bool hold);

  void OnTick(units::time::second_t dt) override;
 private:
  Shooter &_shooter;
  bool _hold;
  units::rad_per_s _speed;
  PIDController _pid;
};

// ShooterSpinup.cpp
#include "ShooterSpinup.h"

using namespace behaviour;

ShooterSpinup::ShooterSpinup(Shooter &s, units::rad_per_s speed, bool hold) : _shooter(s), _speed(speed), _pid(s.pid_settings), _hold(hold), Behaviour("Shooter Spinup") {
  // By saying 'controls', we say that we're controlling the Shooter
  // to make sure that we take over exclusive control of it. That way,
  // no one else is telling it to do something different.
  Controls(s);
  _pid.SetSetpoint(_speed);
}

void ShooterSpinup::OnTick(units::time::second_t dt) {
  // Get the current speed from the shooter's encoder
  units::rad_per_s current_speed = _shooter.gearbox.encoder.GetAngularVelocity();
  // Calculate the feedforward voltage - the voltage required to spin the
  // motor assuming there is no torque (load) applied. This is simple if
  // we use WPILib's DcMotor class from wpimath (#include <frc/system/plant/DcMotor.h>).
  units::volt_t feed_forward = _shooter.gearbox.motor.Voltage(0, _speed);

  // Calculate the PID output from the current speed, time difference, and feed forward.
  units::volt_t pid_demand = _pid.Calculate(current_speed, dt, feed_forward);
  _shooter.SetVoltage(pid_demand);

  // If the PID says we're done, stop this behaviour and move on!
  // Note "hold", which will keep the behaviour running even after we meet our speed
  // We can use this in conjunction with ->Until(behaviour), which will keep our
  // behaviour running until another behaviour is finished.
  if (_pid.IsDone() && !_hold)
    SetDone();
}
```
See how we've taken a complex action and broken it down into a single piece of code that we can reuse? Just have a look how much easier it is to tell the shooter to spinup to an arbitrary speed:
```cpp
auto spinup = make<ShooterSpinup>(my_shooter, 500_rpm, false);
```

We can also assign a timeout to this behaviour, in case we want to move on if it's not ready in time:
```cpp
spinup->WithTimeout(3_s);
```
We can also tell this behaviour to run faster by calling `SetPeriod`:
```cpp
spinup->SetPeriod(10_ms);   // 100Hz
```

## Using Behaviours Together
As we mentioned, Behaviours are small, compartmentalised units of work that we can use together to make complex routines. In order to achieve this, Wombat provides some ways to combine behaviours together into larger sequences.

### Sequential Execution
Behaviours can be executed in sequence by using the `<<` operator.

```cpp
auto combined = make<Behaviour1>()
                << make<Behaviour2>();
// or
auto b1 = make<Behaviour1>();
auto b2 = make<Behaviour2>();
auto combined = b1 << b2;
```

### Parallel Execution
Behaviours can be executed together (at the same time) by using the `&` or `|` operators. Only behaviours which control different systems can be run at the same time. `&` will run until both are complete, whereas `|` will stop after either is complete (known as a "race").

```cpp
auto wait_both = make<Behaviour1>() & make<Behaviour2>();
auto wait_either = make<Behaviour1>() | make<Behaviour2>();
```

You can also run in parallel, waiting until a specific behaviour is complete with the `Until` function.
```cpp
auto wait_until = make<Behaviour1>()
                  ->Until(make<BehaviourDeadline>());
```
### Waiting
Wombat provides `WaitTime` and `WaitFor` to produce simple waits in the behaviour chain.
```cpp
auto wait_2s = make<WaitTime>(2_s);
// WaitFor will wait until a function (predicate) returns true before continuing.
auto wait_until_vision = make<WaitFor>([&vision]() { return vision.ready(); });
```

### Making Decisions
Making decisions in the behaviour chain is easy, as Wombat provides `If`, `Switch`, and `Decide`.

```cpp
auto if_bhvr = make<If>(my_bool)
                ->Then(make<Behaviour1>())
                ->Else(make<Behaviour2>());
// or, with a predicate that's evaluated when the behaviour runs
auto if_bhvr = make<If>([&vision]() { return vision.ready(); })
                ->Then(make<Behaviour1>())
                ->Else(make<Behaviour2>());
```
Switch is similar to a switch-case statement, allowing you to choose from one of many options.
```cpp
auto switch_bhvr = make<Switch>(my_int)
                    // Select based on the value directly
                    ->When(1, make<Behaviour1>())
                    ->When(2, make<Behaviour2>())
                    // Or, based on the value using a predicate
                    ->When([](auto my_int) { return my_int > 6; }, make<Behaviour3>())
                    ->Otherwise(make<Behaviour4>());
// If no When matches, and Otherwise is not provided, the behaviour will
// keep running until one of the options matches. You can also provide
// Otherwise without an argument to exit if none match.
// Like If, you can also provide a function to yield the initial value
auto switch_bhvr = make<Switch>([]() { return my_val; })
  // ...
```

Decide is a special case of Switch, but without an argument - using predicates on all branches.

```cpp
auto decide = make<Decide>()
                ->When([]() { return true; }, make<Behaviour1>())
                ->When([]() { return false; }, make<Behaviour2>());
```
## Big picture: designing Behaviours from the Top Down.
Let's say we come up with a plan to do a really awesome (but really complicated) autonomous. The team decides the following routine is our best strategic option:
- While spinning up the shooter:
  - Move forward 2m
  - Intake a ball
- Shoot the ball
- Turn 90 degrees
- Wait 2 seconds
- Move forward 1.5m
- Intake a ball
- While driving backwards 3m:
  - Spinup the shooter
  - Wait until vision is ready
  - Shoot a ball
- Intake another ball
- Spinup & Shoot the last ball
Complex, right? Let's look at how we break it down. First of all, notice that it's already in a sequence of steps for us - small chunks of work that we can harness to complete our goals. Also notice that there's some common behaviour across this routine - there's multiple times where we move forward, intake a ball, etc. We can use this to our advantage by reducing the amount of code we need to write.
We can use the steps we've already outlined to build our overall behaviour sequence that describes the autonomous routine.
Let's go ahead and mock up what we think our autonomous routine above will look like in code:
```cpp
Behaviour::ptr MyAutoRoutine() {
  return (
    make<ShooterSpinup>(shooter, 500_rpm, true)
      ->Until(
          make<DriveStraight>(drivetrain, 2_m)
          << make<IntakeOne>(intake)
      ))
    << make<ShooterFire>(shooter)
    << make<DriveTurn>(drivetrain, 90_deg)
    << make<WaitTime>(2_s)
    << make<DriveStraight>(drivetrain, 1.5_m)
    << make<IntakeOne>(intake)
    <<  (
          // Drive Straight backwards, and
          make<DriveStraight>(drivetrain, -3_m) &
          (
            // Spinup the shooter until vision is ready, then fire.
            make<ShooterSpinup>(shooter, 500_rpm, true)
              ->Until(make<WaitFor>([vision]() { return vision.ready(); }))
            << make<ShooterFire>(shooter)
          )
        )
    << make<IntakeOne>(intake)
    << make<ShooterSpinup>(shooter, 500_rpm, false)
    << make<ShooterFire>(shooter);
}
```

See how we can just flow on from the individual steps of our big, complex example? Now, instead of implementing 15 different steps in the autonomous routine, we only have to implement 5 behaviours:
- `ShooterSpinup`
- `ShooterFire`
- `DriveStraight`
- `DriveTurn`
- `IntakeOne`
