/*
  Copyright 2019 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: servo_input_parser_client.hpp
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Matthew Piccoli
  Contributors: Raphael Van Hoffelen
*/

#ifndef SERVO_INPUT_PARSER_CLIENT_HPP_
#define SERVO_INPUT_PARSER_CLIENT_HPP_

#include "client_communication.hpp"

const uint8_t kTypeServoInputParser  =   78;

class ServoInputParserClient: public ClientAbstract{
  public:
    ServoInputParserClient(uint8_t obj_idn):
      ClientAbstract( kTypeServoInputParser, obj_idn),
      mode_(          kTypeServoInputParser, obj_idn, kSubMode),
      unit_min_(      kTypeServoInputParser, obj_idn, kSubUnitMin),
      unit_max_(      kTypeServoInputParser, obj_idn, kSubUnitMax)
      {};

    // Client Entries
    // Control commands
    ClientEntry<uint8_t>  mode_;
    ClientEntry<float>    unit_min_;
    ClientEntry<float>    unit_max_;

    uint16_t GetNumberOfClientEntries(){
      return kSubUnitMax + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &mode_,     // 0
        &unit_min_, // 1
        &unit_max_  // 2
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

  private:
    static const uint8_t kSubMode =     0;
    static const uint8_t kSubUnitMin =  1;
    static const uint8_t kSubUnitMax =  2;
};

#endif /* SERVO_INPUT_PARSER_CLIENT_HPP_ */
