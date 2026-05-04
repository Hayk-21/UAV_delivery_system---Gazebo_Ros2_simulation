/*
  Copyright 2024 Vertiq, Inc support@vertiq.co

  This file is part of the Vertiq C++ API.

  This code is licensed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT for details)
*/

#ifndef RGB_LED_CLIENT_H
#define RGB_LED_CLIENT_H

#include "client_communication.hpp"

const uint8_t kTypeRgbLed = 100;

class RgbLedClient : public ClientAbstract {
   public:
    RgbLedClient(uint8_t obj_idn)
        : ClientAbstract(kTypeRgbLed, obj_idn),
          red_(kTypeRgbLed, obj_idn, kSubRed),
          green_(kTypeRgbLed, obj_idn, kSubGreen),
          blue_(kTypeRgbLed, obj_idn, kSubBlue),
          update_color_(kTypeRgbLed, obj_idn, kSubUpdateColor),
          strobe_active_(kTypeRgbLed, obj_idn, kSubStrobeActive),
          strobe_period_(kTypeRgbLed, obj_idn, kSubStrobePeriod),
          strobe_pattern_(kTypeRgbLed, obj_idn, kSubStrobePattern){};

    // Client Entries
    ClientEntry<uint8_t> red_;
    ClientEntry<uint8_t> green_;
    ClientEntry<uint8_t> blue_;
    ClientEntryVoid update_color_;
    ClientEntry<uint8_t> strobe_active_;
    ClientEntry<float> strobe_period_;
    ClientEntry<uint32_t> strobe_pattern_;

    uint16_t GetNumberOfClientEntries(){
      return kSubStrobePattern + 1;
    }

    void GetClientEntryList(ClientEntryAbstract ** client_entries){
      uint16_t num_entries = GetNumberOfClientEntries();

      ClientEntryAbstract* entry_array[num_entries] = {
        &red_,              // 0
        &green_,            // 1
        &blue_,             // 2
        &update_color_,     // 3
        &strobe_active_,    // 4
        &strobe_period_,    // 5
        &strobe_pattern_    // 6
      };

      for(uint16_t entry = 0; entry < num_entries; entry++){
        client_entries[entry] = entry_array[entry];
      }
    }

   private:
    static const uint8_t kSubRed              = 0;
    static const uint8_t kSubGreen            = 1;
    static const uint8_t kSubBlue             = 2;
    static const uint8_t kSubUpdateColor      = 3;
    static const uint8_t kSubStrobeActive     = 4;
    static const uint8_t kSubStrobePeriod     = 5;
    static const uint8_t kSubStrobePattern    = 6;

};

#endif  // RGB_LED_CLIENT_H
