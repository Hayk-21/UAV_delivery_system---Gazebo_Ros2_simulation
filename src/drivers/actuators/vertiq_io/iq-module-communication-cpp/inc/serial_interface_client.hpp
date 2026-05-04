/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: serial_interface_client.hpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#ifndef SERIAL_INTERFACE_CLIENT_H
#define SERIAL_INTERFACE_CLIENT_H

#include "client_communication.hpp"

const uint8_t kTypeSerialInterface = 16;

class SerialInterfaceClient: public ClientAbstract{
  public:
    SerialInterfaceClient(uint8_t obj_idn):
      ClientAbstract(kTypeSerialInterface, obj_idn),
      baud_rate_(    kTypeSerialInterface, obj_idn, kSubBaudRate)
      {};

    // Client Entries
    ClientEntry<uint32_t> baud_rate_;

    uint16_t GetNumberOfClientEntries(){
      return kSubBaudRate + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &baud_rate_,     // 0
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

  private:
    static const uint8_t kSubBaudRate = 0;
};

#endif // SERIAL_INTERFACE_CLIENT_H
