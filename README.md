CurtinFRC 2024 Codebase
===
Our code for the 2024 FRC game, CRESCENDO, using GradleRIO, Wombat, and probably other stuff. Contains two key sections, src/ where all our robot code is stored and wombat/ where our teams library Wombat is stored. To get started follow the setup instructions and read more in [CONTRIBUTING.md](./CONTRIBUTING.md).

Setup
===
First install WPILib and if running Windows the FRC game tools. Instructions can be found [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html).

Fork this repository then open up a terminal and run :
```bash
git clone https://github.com/*yourusernamehere*/2024-Crescendo.git
cd 2024-Crescendo
chmod +x init.sh
./init.sh
```
Now look in [CONTRIBUTING.md](./CONTRIBUTING.md) before continuing!

Windows
---
Fork this repository then open up a terminal and run :
```powershell
git clone https:\\github.com\*yourusernamehere*\2024-Crescendo.git
cd 2024-Crescendo
.\init
```
Now look in [CONTRIBUTING.md](./CONTRIBUTING.md) before continuing!

Quick Commands
===
These commands can be used in a variety of combinations, feel free to experiment!

Build
---
`./gradlew build`
Build will compile your code without deploying it. It will also run all automated tests, which is great for testing code before it runs on a robot.

`./gradlew :wombat:build`
Will compile and build the Wombat library. Also runs all of Wombat's inbuilt tests.

Deploy
---
`./gradlew deploy`
Deploying will build your code (as above), and deploy it to the robot. You have to be connected to the robot for this to work. Just keep in mind that deploying does not run any automated tests

Clean
---
`./gradlew clean`
Cleaning removes caches of your compiled code. If you do not understand an error it can often help to clean before getting help. Clean building is slower so you should not generally use it.

Simulation
----------
**Release**
`./gradlew :simulateNative`
Runs a simulation of your code at highest optimisation.

**Debug**
`./gradlew :simulateNativeDebug`
Runs a debug simulation of your code, including a variety of debugging tools similar to glass but at lower optimisation.

Documentation
=============
Our documentation for Wombat and each years codebase can be found [here](https://4788-docs.vercel.app/). It is a powerful reference for writing code using Wombat going over how to use and the implementation for everything within Wombat. It also has a variety of guides to teach you how to use a variety of things in a more tutorial style way.
