/*
  Copyright 2023 IQinetics Technologies, Inc support@iq-control.com

  This file is part of the IQ C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

/*
  Name: adc_interface_client.hpp
  Last update: 2023/04/18 by Ben Quan
  Author: Ben Quan
  Contributors:
*/

#ifndef ADC_INTERFACE_CLIENT_HPP_
#define ADC_INTERFACE_CLIENT_HPP_

#include "client_communication.hpp"

const uint8_t kTypeAdcInterface = 91;

class AdcInterfaceClient : public ClientAbstract {
   public:
    AdcInterfaceClient(uint8_t obj_idn)
        : ClientAbstract(kTypeAdcInterface, obj_idn),
          adc_voltage_(kTypeAdcInterface, obj_idn, kSubAdcVoltage),
          raw_value_(kTypeAdcInterface, obj_idn, kSubRawValue){};

    // Client Entries
    ClientEntry<float> adc_voltage_;
    ClientEntry<uint16_t> raw_value_;

    uint16_t GetNumberOfClientEntries(){
      return kSubRawValue + 1;
    }

   void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
          &adc_voltage_,  // 0
          &raw_value_     // 1
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
   }

   private:
    static const uint8_t kSubAdcVoltage = 0;
    static const uint8_t kSubRawValue   = 1;
};

#endif /* ADC_INTERFACE_CLIENT_HPP_ */
