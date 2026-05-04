/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: step_direction_input_client.hpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#ifndef STEP_DIRECTION_INPUT_CLIENT_HPP_
#define STEP_DIRECTION_INPUT_CLIENT_HPP_


#include "client_communication.hpp"

const uint8_t kTypeStepDirInput = 58;

class StepDirectionInputClient: public ClientAbstract{
  public:
    StepDirectionInputClient(uint8_t obj_idn):
      ClientAbstract( kTypeStepDirInput, obj_idn),
      angle_(         kTypeStepDirInput, obj_idn, kSubAngle),
      angle_step_(    kTypeStepDirInput, obj_idn, kSubAngleStep)
      {};

    // Client Entries
    ClientEntry<float>      angle_;
    ClientEntry<float>      angle_step_;

    uint16_t GetNumberOfClientEntries(){
      return kSubAngleStep + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &angle_,      // 0
        &angle_step_  // 1
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

  private:
    static const uint8_t kSubAngle               = 0;
    static const uint8_t kSubAngleStep           = 1;
};

#endif /* STEP_DIRECTION_INPUT_CLIENT_HPP_ */
