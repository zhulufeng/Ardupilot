// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Lidar804.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_Lidar804::AP_RangeFinder_Lidar804(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_Lidar804::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_Lidar804::get_reading(uint16_t &reading_cm,int16_t &velvelocity_cm)
{
    if (uart == nullptr) {
        return false;
    }
    bool     ret        = false;
    uint16_t dist_sum   = 0;
    uint16_t dist_cnt   = 0;
    float    vel_sum    = 0;
    
    int      numc       = 0;
    uint8_t  data;
    bool     data_valid = false;
    // read any available lines from the lidar
    float     sum    = 0;
    uint16_t  count  = 0;
    int16_t   nbytes = uart->available();
    while (nbytes-- > 0) {
        data = uart->read();            
        switch(step) {
          case 0:
            if (data == 0x55)
            {//judge mag frame header second byte
                check_sum     = 0;
                radar_counter = 0;
                step++;
            }
            break;
                
          case 1:
            radar_bytes[radar_counter] = data;
            check_sum                  += data;
            if(++ radar_counter >= 13){
              if(radar_bytes[4] != 0x00){
                step++;
                //hal.console->printf_P(PSTR("data %d\n"),radar_bytes[4]);
                }
              else{
                step = 0;
                }
            }
            break;
          case 2:
            if(check_sum == data)
              step++;
            else
              step = 0;
            break;
          case 3:
            if(data == 0xAA)
            {
              dist_sum += ((((uint16_t)radar_bytes[1]) << 8) + (((uint16_t)radar_bytes[0])));
              vel_sum += -(( (((int32_t)radar_bytes[3]) << 24) + (((int32_t)radar_bytes[2]) << 16) )/65536);
              dist_cnt ++;
              //data_valid = (radar_bytes[4] == 0x00)? false:true;
              //al.console->printf_P(PSTR("data_valid %d\n"),radar_bytes[4]);
              //hal.console->printf_P(PSTR("dist_cnt %d\n"),dist_cnt);
            }
              //hal.console->printf_P(PSTR("dist_sum %d\n"),dist_sum);
            step = 0;
            break;
        }//end of switch
    }//end of while
    if(dist_cnt >0)
    {
      dist      = dist_sum * 10 / dist_cnt;
      ra_vel    = vel_sum/ dist_cnt;

      //dist_time = hrt_absolute_time();
      ret       = true;
    }
    


    // we need to write a byte to prompt another reading
    //uart->write('\n');

    if (dist_cnt == 0) {
      return false;
    }
    reading_cm     = dist / 10.0;
    velvelocity_cm = ra_vel;
    hal.console -> printf_P("reading_cm :  %d  velvelocity_cm %d \n", reading_cm,velvelocity_cm);

    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_Lidar804::update(void)
{
    if (get_reading(state.distance_cm,state.velvelocity_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = hal.scheduler->millis();
        update_status();
    } else if (hal.scheduler->millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
