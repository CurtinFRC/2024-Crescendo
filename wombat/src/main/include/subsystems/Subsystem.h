#pragma once

#include "behaviour/HasBehaviour.h"

#include <units/time.h>

namespace wom {
  template<typename ConfigType, typename StateType>
  class Subsystem: public behaviour::HasBehaviour {
   public:
    Subsystem(ConfigType *config);
    ~Subsystem();

    ConfigType *getConfig();
    StateType getState();

    void setState(StateType state);

    virtual void OnStart();
    virtual void OnUpdate(units::second_t dt);

   protected:
    ConfigType *_config;
    StateType _state;

   private:
  };
};

