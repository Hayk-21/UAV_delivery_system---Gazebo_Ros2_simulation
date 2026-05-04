/*
  Copyright 2023 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: stopping_handler_client.hpp
  Last update: 2023/04/12 by Ben Quan
  Author: Ben Quan
  Contributors:
*/

#ifndef STOPPING_HANDLER_CLIENT_HPP_
#define STOPPING_HANDLER_CLIENT_HPP_

#include "client_communication.hpp"

const uint8_t kTypeStoppingHandler = 87;

class StoppingHandlerClient : public ClientAbstract {
   public:
    StoppingHandlerClient(uint8_t obj_idn)
        : ClientAbstract(kTypeStoppingHandler, obj_idn),
          stopped_speed_(kTypeStoppingHandler, obj_idn, kSubStoppedSpeed),
          stopped_time_(kTypeStoppingHandler, obj_idn, kSubStoppedTime){};

    // Client Entries
    ClientEntry<float> stopped_speed_;
    ClientEntry<float> stopped_time_;

    uint16_t GetNumberOfClientEntries(){
      return kSubStoppedTime + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
          &stopped_speed_,  // 0
          &stopped_time_    // 1
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

   private:
    static const uint8_t kSubStoppedSpeed = 0;
    static const uint8_t kSubStoppedTime  = 1;
};

#endif /* STOPPING_HANDLER_CLIENT_HPP_ */
