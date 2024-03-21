// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "behaviour/TriggerXboxController.h"

#include "utils/Util.h"

using namespace behaviour;

TriggerXboxController::TriggerXboxController(int port) : m_hid{port} {}

frc::XboxController& TriggerXboxController::GetHID() {
  return m_hid;
}

Trigger* TriggerXboxController::AButton() {
  return new Trigger([this]() { return m_hid.GetAButton(); }, "A button trigger");
}

Trigger* TriggerXboxController::AButtonReleased() {
  return new Trigger([this]() { return m_hid.GetBButtonReleased(); }, "A button released trigger");
}

Trigger* TriggerXboxController::AButtonPressed() {
  return new Trigger([this]() { return m_hid.GetAButtonPressed(); }, "A button pressed trigger");
}

Trigger* TriggerXboxController::BButton() {
  return new Trigger([this]() { return m_hid.GetBButton(); }, "B button trigger");
}

Trigger* TriggerXboxController::BButtonReleased() {
  return new Trigger([this]() { return m_hid.GetBButtonReleased(); }, "B button released trigger");
}

Trigger* TriggerXboxController::BButtonPressed() {
  return new Trigger([this]() { return m_hid.GetBButtonPressed(); }, "B button pressed trigger");
}

Trigger* TriggerXboxController::XButton() {
  return new Trigger([this]() { return m_hid.GetXButton(); }, "X button trigger");
}

Trigger* TriggerXboxController::XButtonReleased() {
  return new Trigger([this]() { return m_hid.GetXButtonReleased(); }, "X button released trigger");
}

Trigger* TriggerXboxController::XButtonPressed() {
  return new Trigger([this]() { return m_hid.GetXButtonPressed(); }, "X button pressed trigger");
}

Trigger* TriggerXboxController::YButton() {
  return new Trigger([this]() { return m_hid.GetYButton(); }, "Y button trigger");
}

Trigger* TriggerXboxController::YButtonReleased() {
  return new Trigger([this]() { return m_hid.GetYButtonReleased(); }, "Y button released trigger");
}

Trigger* TriggerXboxController::YButtonPressed() {
  return new Trigger([this]() { return m_hid.GetYButtonPressed(); }, "Y button pressed trigger");
}

Trigger* TriggerXboxController::StartButton() {
  return new Trigger([this]() { return m_hid.GetStartButton(); }, "Start button trigger");
}

Trigger* TriggerXboxController::StartButtonReleased() {
  return new Trigger([this]() { return m_hid.GetStartButtonReleased(); }, "Start button released trigger");
}

Trigger* TriggerXboxController::StartButtonPressed() {
  return new Trigger([this]() { return m_hid.GetStartButtonPressed(); }, "Start button pressed trigger");
}

Trigger* TriggerXboxController::BackButton() {
  return new Trigger([this]() { return m_hid.GetBackButton(); }, "Back button trigger");
}

Trigger* TriggerXboxController::BackButtonReleased() {
  return new Trigger([this]() { return m_hid.GetBackButtonReleased(); }, "Back button released trigger");
}

Trigger* TriggerXboxController::BackButtonPressed() {
  return new Trigger([this]() { return m_hid.GetBackButtonPressed(); }, "Back button pressed trigger");
}

Trigger* TriggerXboxController::LeftStickButton() {
  return new Trigger([this]() { return m_hid.GetLeftStickButton(); }, "Left Stick button trigger");
}

Trigger* TriggerXboxController::LeftStickButtonReleased() {
  return new Trigger([this]() { return m_hid.GetLeftStickButtonReleased(); },
                     "Left Stick button released trigger");
}

Trigger* TriggerXboxController::LeftStickButtonPressed() {
  return new Trigger([this]() { return m_hid.GetLeftStickButtonPressed(); },
                     "Left Stick button pressed trigger");
}

Trigger* TriggerXboxController::RightStickButton() {
  return new Trigger([this]() { return m_hid.GetRightStickButton(); }, "Right stick button trigger");
}

Trigger* TriggerXboxController::RightStickButtonReleased() {
  return new Trigger([this]() { return m_hid.GetRightStickButtonReleased(); },
                     "Right Stick button released trigger");
}

Trigger* TriggerXboxController::RightStickButtonPressed() {
  return new Trigger([this]() { return m_hid.GetRightStickButtonPressed(); },
                     "Right Stick button pressed trigger");
}

Trigger* TriggerXboxController::LeftBumper() {
  return new Trigger([this]() { return m_hid.GetLeftBumper(); }, "Left bumper trigger");
}

Trigger* TriggerXboxController::LeftBumperReleased() {
  return new Trigger([this]() { return m_hid.GetLeftBumperReleased(); }, "Left bumper released trigger");
}

Trigger* TriggerXboxController::LeftBumperPressed() {
  return new Trigger([this]() { return m_hid.GetLeftBumperPressed(); }, "Left bumper pressed trigger");
}

Trigger* TriggerXboxController::RightBumper() {
  return new Trigger([this]() { return m_hid.GetRightBumper(); }, "Right bumper trigger");
}

Trigger* TriggerXboxController::RightBumperReleased() {
  return new Trigger([this]() { return m_hid.GetRightBumperReleased(); }, "Right bumper released trigger");
}

Trigger* TriggerXboxController::RightBumperPressed() {
  return new Trigger([this]() { return m_hid.GetRightBumperPressed(); }, "Right bumper pressed trigger");
}

Trigger* TriggerXboxController::RightTriggerThreshold(double threshold) {
  return new Trigger([this]() { return GetRightTriggerAxis() > 0.05; }, "Right Trigger threshold trigger");
}

Trigger* TriggerXboxController::LeftTriggerThreshold(double threshold) {
  return new Trigger([this]() { return GetLeftTriggerAxis() > 0.05; }, "Left Trigger threshold trigger");
}

Trigger* TriggerXboxController::RightJoystickXThreshold(double threshold) {
  return new Trigger([this]() { return GetRightX() > 0.05; }, "Right Joystick X threshold trigger");
}

Trigger* TriggerXboxController::RightJoystickYThreshold(double threshold) {
  return new Trigger([this]() { return GetRightY() > 0.05; }, "Right Joystick Y threshold trigger");
}

Trigger* TriggerXboxController::LeftJoystickXThreshold(double threshold) {
  return new Trigger([this]() { return GetRightX() > 0.05; }, "Left Joystick X threshold trigger");
}

Trigger* TriggerXboxController::LeftJoystickYThreshold(double threshold) {
  return new Trigger([this]() { return GetRightY() > 0.05; }, "Left Joystick Y threshold trigger");
}

double TriggerXboxController::GetRightTriggerAxis() {
  return m_hid.GetRightTriggerAxis();
}

double TriggerXboxController::GetLeftTriggerAxis() {
  return m_hid.GetLeftTriggerAxis();
}

double TriggerXboxController::GetRightY() {
  return m_hid.GetRightY();
}

double TriggerXboxController::GetLeftY() {
  return m_hid.GetLeftY();
}

double TriggerXboxController::GetRightX() {
  return m_hid.GetRightX();
}

double TriggerXboxController::GetLeftX() {
  return m_hid.GetLeftX();
}
