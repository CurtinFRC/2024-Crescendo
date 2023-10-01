CurtinFRC 2024 Codebase
===

# Setup
## Linux
Run
```bash
git clone https://github.com/yourusernamehere/2024-Crescendo.git
cd 2024-Crescendo
git clone https://github.com/CurtinFRC/Wombat.git
mv Wombat wombat
rm Wombat/.git -rf
chmod +x gradlew
./gradlew installRoborioToolchain
./gradlew build
```

## Windows
TODO!

# Quick Commands
## Build
`./gradlew build`
Build will compile and get the code ready without deploying it. It will also run all automated tests, which is great for testing your code before it ever gets run on a robot (which also means you can build whenever).  

`./gradlew :Wombat:build`
Will compile and build the Wombat library. Also runs all of Wombat's inbuilt tests.

## Deploy
`./gradlew`
Deploying will build your code (as above), and deploy it to the robot. You have to be connected to the robot for this to work. Just keep in mind that deploying _does not run any automated tests

