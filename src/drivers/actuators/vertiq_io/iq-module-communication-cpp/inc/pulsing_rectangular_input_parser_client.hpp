/*
  Copyright 2023 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: pulsing_rectangular_input_parser_client.hpp
  Last update: 2023/04/12 by Ben Quan
  Author: Ben Quan
  Contributors:
*/

#ifndef PULSING_RECTANGULAR_INPUT_PARSER_CLIENT_HPP_
#define PULSING_RECTANGULAR_INPUT_PARSER_CLIENT_HPP_

#include "client_communication.hpp"

const uint8_t kTypePulsingRectangularInputParser = 89;

class PulsingRectangularInputParserClient : public ClientAbstract {
   public:
    PulsingRectangularInputParserClient(uint8_t obj_idn)
        : ClientAbstract(kTypePulsingRectangularInputParser, obj_idn),
          pulsing_voltage_mode_(kTypePulsingRectangularInputParser, obj_idn, kSubPulsingVoltageMode),
          pulsing_voltage_limit_(kTypePulsingRectangularInputParser, obj_idn, kSubPulsingVoltageLimit)
          {
          };

    // Client Entries
    ClientEntry<uint8_t> pulsing_voltage_mode_;
    ClientEntry<float> pulsing_voltage_limit_;

    uint16_t GetNumberOfClientEntries(){
      return kSubPulsingVoltageLimit + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &pulsing_voltage_mode_,  // 0
        &pulsing_voltage_limit_  // 1
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

   private:
    static const uint8_t kSubPulsingVoltageMode  = 0;
    static const uint8_t kSubPulsingVoltageLimit = 1;
};

#endif /* PULSING_RECTANGULAR_INPUT_PARSER_CLIENT_HPP_ */
