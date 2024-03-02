// // Copyright (c) 2023-2024 CurtinFRC
// // Open Source Software, you can modify it according to the terms
// // of the MIT License at the root of this project

// #include "Mag.h"

// Mag::Mag(MagConfig config) : _config(config) {}

// void Mag::OnUpdate(units::second_t dt) {
//   switch (_state) {
//     case MagState::kIdle: {
//       if (_config.intakeSensor->Get()) {
//         SetState(MagState::kHold);
//       } else if (_config.intakeSensor->Get()) {
//         SetState(MagState::kHold);
//       }
//       _setVoltage = 0_V;
//       _stringStateName = "Idle";
//     } break;

//     case MagState::kHold: {
//       if (_config.magSensor->Get() == 0) {
//         SetState(MagState::kIdle);
//       }
//       _setVoltage = 0_V;
//       _stringStateName = "Hold";
//     } break;

//     case MagState::kEject: {
//       if (_config.magSensor->Get() == 0 && _config.intakeSensor->Get() == 0) {
//         SetState(MagState::kIdle);
//       }
//       _setVoltage = -5_V;
//       _stringStateName = "Eject";
//     } break;

//     case MagState::kRaw:
//       _setVoltage = _rawVoltage;
//       _stringStateName = "Raw";
//       break;

//     case MagState::kPass: {
//       if (_config.shooterSensor->Get()) {
//         SetState(MagState::kIdle);
//       } else {
//         _setVoltage = 5_V;
//         _stringStateName = "Pass";
//       }
//     } break;

//     default:
//       std::cout << "Error magazine in invalid state" << std::endl;
//       break;
//   }
//   _config.magGearbox.motorController->SetVoltage(_setVoltage);
//   _table->GetEntry("State: ").SetString(_stringStateName);
//   _table->GetEntry("Motor Voltage: ").SetDouble(_setVoltage.value());
//   _table->GetEntry("Intake Sensor: ").SetDouble(_config.intakeSensor->Get());
//   _table->GetEntry("Shooter Sensor: ").SetDouble(_config.shooterSensor->Get());
//   _table->GetEntry("Magazine Sensor: ").SetDouble(_config.magSensor->Get());
// }

// void Mag::SetState(MagState state) {
//   _state = state;
// }

// void Mag::SetRaw(units::volt_t voltage) {
//   _rawVoltage = voltage;
// }
// MagState Mag::GetState() {
//   return _state;
// }
