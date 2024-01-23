#!/usr/bin/sh
sudo apt update
chmod +x gradlew
./gradlew installRoborioToolchain
./gradlew build
