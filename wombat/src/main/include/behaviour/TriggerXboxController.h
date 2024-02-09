// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "behaviour/Trigger.h"

namespace behaviour {
/**
 * @brief A wrapper over frc::XboxController that creates a Trigger for each button.
 *
 * This class provides a wrapper over the frc::XboxController that will create a Trigger for each button and
 * different button action. This allows you to easily bind behaviours to a controller with certainty as to
 * what is being used. This avoids the ambiguity of passing controllers around to different behaviours.
 */
class TriggerXboxController {
 public:
  /**
   * Creates a new TriggerXboxController.
   *
   * @param port The port of the XboxController.
   */
  explicit TriggerXboxController(int port);

  /**
   * Returns the underlying XboxController.
   */
  frc::XboxController& GetHID();

  /**
   * Creates a new Trigger binded to the left bumper of the XboxController.
   */
  Trigger* LeftBumper();

  /**
   * Creates a new Trigger binded to the release of left bumper.
   */
  Trigger* LeftBumperPressed();

  /**
   * Creates a new Trigger binded to the left bumper being pressed.
   */
  Trigger* LeftBumperReleased();

  /**
   * Creates a new Trigger binded to the right bumper of the XboxController.
   */
  Trigger* RightBumper();

  /**
   * Creates a new Trigger binded to the release of right bumper.
   */
  Trigger* RightBumperPressed();

  /**
   * Creates a new Trigger binded to the right bumper being pressed.
   */
  Trigger* RightBumperReleased();

  /**
   * Creates a new Trigger Binded to the left stick button.
   */
  Trigger* LeftStickButton();

  /**
   * Creates a new Trigger Binded to the left stick button being pressed.
   */
  Trigger* LeftStickButtonPressed();

  /**
   * Creates a new Trigger Binded to the left stick button being released.
   */
  Trigger* LeftStickButtonReleased();

  /**
   * Creates a new Trigger Binded to the right stick button.
   */
  Trigger* RightStickButton();

  /**
   * Creates a new Trigger Binded to the right stick button being pressed.
   */
  Trigger* RightStickButtonPressed();

  /**
   * Creates a new Trigger Binded to the right stick button being released.
   */
  Trigger* RightStickButtonReleased();

  /**
   * Creates a new Trigger Binded to the a button.
   */
  Trigger* AButton();

  /**
   * Creates a new Trigger Binded to the a button being pressed.
   */
  Trigger* AButtonPressed();

  /**
   * Creates a new Trigger Binded to the a button being released.
   */
  Trigger* AButtonReleased();

  /**
   * Creates a new Trigger Binded to the b button.
   */
  Trigger* BButton();

  /**
   * Creates a new Trigger Binded to the b button being pressed.
   */
  Trigger* BButtonPressed();

  /**
   * Creates a new Trigger Binded to the b button being released.
   */
  Trigger* BButtonReleased();

  /**
   * Creates a new Trigger Binded to the x button.
   */
  Trigger* XButton();

  /**
   * Creates a new Trigger Binded to the x button being pressed.
   */
  Trigger* XButtonPressed();

  /**
   * Creates a new Trigger Binded to the x button being released.
   */
  Trigger* XButtonReleased();

  /**
   * Creates a new Trigger Binded to the y button.
   */
  Trigger* YButton();

  /**
   * Creates a new Trigger Binded to the y button being pressed.
   */
  Trigger* YButtonPressed();

  /**
   * Creates a new Trigger Binded to the y button being released.
   */
  Trigger* YButtonReleased();

  /**
   * Creates a new Trigger Binded to the back button.
   */
  Trigger* BackButton();

  /**
   * Creates a new Trigger Binded to the back button being pressed.
   */
  Trigger* BackButtonPressed();

  /**
   * Creates a new Trigger Binded to the back button being released.
   */
  Trigger* BackButtonReleased();

  /**
   * Creates a new Trigger Binded to the start button.
   */
  Trigger* StartButton();

  /**
   * Creates a new Trigger Binded to the start button being pressed.
   */
  Trigger* StartButtonPressed();

  /**
   * Creates a new Trigger Binded to the start button being released.
   */
  Trigger* StartButtonReleased();

  /**
   * Get the right trigger (RT) axis value of the controller. Note that this
   * axis is bound to the range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  double GetRightTriggerAxis();

  /**
   * Get the left trigger (LT) axis value of the controller. Note that this axis
   * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  double GetLeftTriggerAxis();

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  double GetRightY();

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  double GetLeftY();

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  double GetRightX();

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  double GetLeftX();

 private:
  frc::XboxController m_hid;
};
}  // namespace behaviour
