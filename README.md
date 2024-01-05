CurtinFRC 2024 Codebase
===
![Build Status of Upstream Master](https://github.com/CurtinFRC/2024-Crescendo/actions/workflows/ci.yml/badge.svg)

Setup
=====

**This assumes that you have the WPILib tools installed. If you do not, follow the instructions [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).**

Linux
-----
Fork this repository then open up a terminal and run :
```bash
git clone https://github.com/*yourusernamehere*/2024-Crescendo.git
cd 2024-Crescendo
chmod +x init.sh
./init.sh
```
Now look in [CONTRIBUTING.md](./CONTRIBUTING.md) before continuing!

Windows
-------
Fork this repository then open up a terminal and run :
```powershell
git clone https:\\github.com\*yourusernamehere*\2024-Crescendo.git
cd 2024-Crescendo
.\init
```
Now look in [CONTRIBUTING.md](./CONTRIBUTING.md) before continuing!

Quick Commands

These commands can be used in a variety of combinations, feel free to experiment!

Build
------
`./gradlew build`
Build will compile and get the code ready without deploying it.
It will also run all automated tests, which is great for testing your code before it ever gets run on a robot (which also means you can build whenever).

`./gradlew :wombat:build`
Will compile and build the Wombat library.
Also runs all of Wombat's inbuilt tests.

Deploy
------
`./gradlew deploy`
Deploying will build your code (as above), and deploy it to the robot.
You have to be connected to the robot for this to work.
Just keep in mind that deploying does not run any automated tests

Clean
-----
`./gradlew clean`
Cleaning removes caches of your compiled code.
If you do not understand an error it can often help to clean before getting help.
Clean building is slower so you should not generally use it.

Our documentation for Wombat and each years codebase can be found [here](TODO!).
=======
Simulation
----------
**Release**
`./gradlew :nativeSimulation`
Runs a simulation of your code.

**Debug**
`./gradlew :nativeSimulationDebug`
Runs a debug simulation of your code, including a variety of debugging tools similar to glass.

Documentation
=============
Our documentation for Wombat and each years codebase can be found [here](https://4788-docs.vercel.app/).
It is a powerful reference for writing code using Wombat going over how to use and the implementation for everything within Wombat.
It also has a variety of guides to teach you how to use a variety of things in a more fully scoped way.
