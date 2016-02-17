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

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "AP_RangeFinder_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_range_finder.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

uint8_t AP_RangeFinder_PX4::num_px4_instances = 0;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PX4::AP_RangeFinder_PX4(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
	AP_RangeFinder_Backend(_ranger, instance, _state),
    _last_max_distance_cm(-1),
    _last_min_distance_cm(-1),
    radar_counter(0),
    port(hal.uartE),
    dist(0),
    step(0),
    check_sum(0),
    ra_vel(0)
{
    port->begin(115200, 256, 16);///
    state.healthy = true;
}

/* 
   close the file descriptor
*/
AP_RangeFinder_PX4::~AP_RangeFinder_PX4()
{

}

/* 
   open the PX4 driver, returning the file descriptor
*/
int AP_RangeFinder_PX4::open_driver(void)
{

}

/* 
   see if the PX4 driver is available
*/
bool AP_RangeFinder_PX4::detect(RangeFinder &_ranger, uint8_t instance)
{
//    int fd = open_driver();
//    if (fd == -1) {
//        return false;
//    }
//    close(fd);
    return true;
}

void AP_RangeFinder_PX4::update(void)
{

    bool valid;
    valid = read_uart_radar();
    if(valid){
    	state.distance_cm = dist/10;
    	 state.radar_vel = ra_vel;
    	_last_timestamp = dist_time;
    }
//    hal.console->printf_P(PSTR("state.radar_vel %f\n"),ra_vel);
//    hal.console->printf_P(PSTR("state.distance_cm %d\n"),state.distance_cm);

     _last_max_distance_cm = 3000;//ranger._max_distance_cm[state.instance];
     _last_min_distance_cm = 90;//ranger._min_distance_cm[state.instance];

    // consider the range finder healthy if we got a reading in the last 0.2s
    state.healthy = (hrt_absolute_time() - _last_timestamp < 200000);


}

bool AP_RangeFinder_PX4::read_uart_radar(void)
{
	bool ret = false;
	uint16_t dist_sum = 0;
	uint16_t dist_cnt = 0;
	float vel_sum = 0;

	int numc = 0;
	uint8_t data;
	numc = port->available();
//	hal.console->printf_P(PSTR("numc %d\n"),numc);
	if (numc > 0) {
			for (int16_t i = 0; i < numc; i++) {
			        // read the next byte
			        data = port->read();
			       hal.console->printf_P(PSTR("data %d\n"),data);
			switch(step) {
			 case 0:
			      if (data == 0x55) {//judge mag frame header second byte
			        	check_sum = 0;
			        	radar_counter = 0;
			        	step++;
			        	}
		        	break;

		    case 1:

	        	radar_bytes[radar_counter] = data;
	        	check_sum += data;
	        	if(++ radar_counter >= 13)
	        		if(radar_bytes[12] == 1)
	        			step++;
	        		else
	        			step = 0;
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
					vel_sum += -(float)(( (((int32_t)radar_bytes[3]) << 24) + (((int32_t)radar_bytes[2]) << 16) )/65536);
					dist_cnt ++;
					//hal.console->printf_P(PSTR("dist_sum %d\n"),dist_sum);
				}
				step = 0;
				break;
			}
	   }
		if(dist_cnt > 0)
		{

			dist = dist_sum * 10 / dist_cnt;
			ra_vel = vel_sum/(float) dist_cnt;
			dist_time = hrt_absolute_time();
			ret = true;

		}
	}
	return ret;
}

#endif // CONFIG_HAL_BOARD
