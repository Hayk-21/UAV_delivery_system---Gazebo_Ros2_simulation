/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: persistent_memory_client.hpp
  Last update: 10/31/2022 by Ben Quan
  Author: Matthew Piccoli
  Contributors: Ben Quan, Raphael Van Hoffelen
*/

#ifndef PERSISTENT_MEMORY_CLIENT_HPP_
#define PERSISTENT_MEMORY_CLIENT_HPP_

#include "client_communication.hpp"

const uint8_t kTypePersistentMemory  =   11;

class PersistentMemoryClient: public ClientAbstract{
  public:
    PersistentMemoryClient(uint8_t obj_idn):
      ClientAbstract(     kTypePersistentMemory, obj_idn),
      erase_(             kTypePersistentMemory, obj_idn, kSubErase),
      revert_to_default_( kTypePersistentMemory, obj_idn, kSubRevertToDefault),
      format_key_1_(      kTypePersistentMemory, obj_idn, kSubFormatKey1),
      format_key_2_(      kTypePersistentMemory, obj_idn, kSubFormatKey2)
      {};

    // Client Entries
    // Control commands
    ClientEntryVoid       erase_;
    ClientEntryVoid       revert_to_default_;
    ClientEntry<uint32_t>  format_key_1_;
    ClientEntry<uint32_t>  format_key_2_;

    uint16_t GetNumberOfClientEntries(){
      return kSubFormatKey2 + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &erase_,             // 0
        &revert_to_default_, // 1
        &format_key_1_,      // 2
        &format_key_2_       // 3
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

  private:
    static const uint8_t kSubErase            = 0;
    static const uint8_t kSubRevertToDefault  = 1;
    static const uint8_t kSubFormatKey1       = 2;
    static const uint8_t kSubFormatKey2       = 3;
};

#endif /* PERSISTENT_MEMORY_CLIENT_HPP_ */
