/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: anticogging_client.hpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#ifndef ANTICOGGING_CLIENT_H
#define ANTICOGGING_CLIENT_H

#include "client_communication.hpp"

const uint8_t kTypeAnticogging = 71;

class AnticoggingClient: public ClientAbstract{
  public:
    AnticoggingClient(uint8_t obj_idn):
      ClientAbstract( kTypeAnticogging, obj_idn),
      table_size_(    kTypeAnticogging, obj_idn, kSubTableSize),
      is_data_valid_( kTypeAnticogging, obj_idn, kSubIsDataValid),
      is_enabled_(    kTypeAnticogging, obj_idn, kSubIsEnabled),
      erase_(         kTypeAnticogging, obj_idn, kSubErase),
      left_shift_(    kTypeAnticogging, obj_idn, kSubLeftShift)
      {};

    // Client Entries
    ClientEntry<uint16_t> table_size_;
    ClientEntry<uint8_t> is_data_valid_;
    ClientEntry<uint8_t> is_enabled_;
    ClientEntryVoid erase_;
    ClientEntry<uint8_t> left_shift_;


    uint16_t GetNumberOfClientEntries(){
      return kSubLeftShift + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &table_size_,     // 0
        &is_data_valid_,  // 1
        &is_enabled_,     // 2
        &erase_,          // 3
        &left_shift_      // 4
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

  private:
    static const uint8_t kSubTableSize    = 0;
    static const uint8_t kSubIsDataValid  = 1;
    static const uint8_t kSubIsEnabled    = 2;
    static const uint8_t kSubErase        = 3;
    static const uint8_t kSubLeftShift    = 4;
};

#endif // ANTICOGGING_CLIENT_H
