#!/usr/bin/sh
sudo apt update
sudo apt install -y openjdk-11-jre-headless
chmod +x gradlew
./gradlew installRoborioToolchain
./gradlew build

