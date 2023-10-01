#include "subsystems/Subsystem.h"

using namespace wom;

template<typename ConfigType, typename StateType>
Subsystem<ConfigType, StateType>::Subsystem(ConfigType *config): _config(config) {}

template<typename ConfigType, typename StateType>
Subsystem<ConfigType, StateType>::~Subsystem() {}

template<typename ConfigType, typename StateType>
ConfigType *Subsystem<ConfigType, StateType>::getConfig() { return _config; }

template<typename ConfigType, typename StateType>
StateType Subsystem<ConfigType, StateType>::getState() { return _state; }

template<typename ConfigType, typename StateType>
void Subsystem<ConfigType, StateType>::setState(StateType state) { _state = state; }
 
