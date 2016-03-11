/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_InertialNav.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav::var_info[] PROGMEM = {
    // start numbering at 1 because 0 was previous used for body frame accel offsets
    // @Param: TC_XY
    // @DisplayName: Horizontal Time Constant
    // @Description: Time constant for GPS and accel mixing. Higher TC decreases GPS impact on position estimate
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_XY",   1, AP_InertialNav, _time_constant_xy, AP_INTERTIALNAV_TC_XY),

    // @Param: TC_Z
    // @DisplayName: Vertical Time Constant
    // @Description: Time constant for baro and accel mixing. Higher TC decreases barometers impact on altitude estimate
    // @Range: 0 10
    // @Increment: 0.1
    AP_GROUPINFO("TC_Z",    2, AP_InertialNav, _time_constant_z, AP_INTERTIALNAV_TC_Z),

    AP_GROUPEND
};




// init - initialise library
void AP_InertialNav::init()
{
    // recalculate the gains
    update_gains();
	rng_valid = false;
}

// update - updates velocities and positions using latest info from ahrs and barometer if new data is available;
void AP_InertialNav::update(float dt)
{
    // discard samples where dt is too large
    static uint32_t loop = 0;
	loop++;
	last_update_time = hal.scheduler->millis();
    if( dt > 0.1f ) {
        return;
    }

    // decrement ignore error count if required
    if (_flags.ignore_error > 0) {
        _flags.ignore_error--;
    }

    // check if new baro readings have arrived and use them to correct vertical accelerometer offsets.
    check_baro();

    // check if new gps readings have arrived and use them to correct position estimates
    check_gps();
	rng_inav_check();
    Vector3f accel_ef = _ahrs.get_accel_ef();

    // remove influence of gravity
    accel_ef.z += GRAVITY_MSS;
    accel_ef *= 100.0f;

    // remove xy if not enabled
    if( !_xy_enabled ) {
        accel_ef.x = 0.0f;
        accel_ef.y = 0.0f;
    }

    //Convert North-East-Down to North-East-Up
    accel_ef.z = -accel_ef.z;

    // convert ef position error to horizontal body frame
    Vector2f position_error_hbf;
    position_error_hbf.x = _position_error.x * _ahrs.cos_yaw() + _position_error.y * _ahrs.sin_yaw();
    position_error_hbf.y = -_position_error.x * _ahrs.sin_yaw() + _position_error.y * _ahrs.cos_yaw();

    float tmp = _k3_xy * dt;
    accel_correction_hbf.x += position_error_hbf.x * tmp;
    accel_correction_hbf.y += position_error_hbf.y * tmp;
    accel_correction_hbf.z += _position_error.z * _k3_z  * dt;

    tmp = _k2_xy * dt;
    _velocity.x += _position_error.x * tmp;
    _velocity.y += _position_error.y * tmp;
    _velocity.z += _position_error.z * _k2_z  * dt;

    tmp = _k1_xy * dt;
    _position_correction.x += _position_error.x * tmp;
    _position_correction.y += _position_error.y * tmp;
    _position_correction.z += _position_error.z * _k1_z  * dt;

    // convert horizontal body frame accel correction to earth frame
    Vector2f accel_correction_ef;
    accel_correction_ef.x = accel_correction_hbf.x * _ahrs.cos_yaw() - accel_correction_hbf.y * _ahrs.sin_yaw();
    accel_correction_ef.y = accel_correction_hbf.x * _ahrs.sin_yaw() + accel_correction_hbf.y * _ahrs.cos_yaw();

    // calculate velocity increase adding new acceleration from accelerometers
    Vector3f velocity_increase;
    velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
    velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
    velocity_increase.z = (accel_ef.z + accel_correction_hbf.z) * dt;

    // calculate new estimate of position
    _position_base += (_velocity + velocity_increase*0.5) * dt;

    // update the corrected position estimate
    _position = _position_base + _position_correction;

    // calculate new velocity
    _velocity += velocity_increase;
	if(loop == 10){
		hal.console->printf_P(PSTR("position.z:  %f\t"),_position.z);
		hal.console->printf_P(PSTR("rng_alt in update %f\n"),(float)_rng_alt);
		//hal.console->printf_P(PSTR("_position_base.z:  %f\t"),_position_base.z);
		//hal.console->printf_P(PSTR("_position_correction.z:  %f\n"),_position_correction.z);
		loop = 0;
		}
    // store 3rd order estimate (i.e. estimated vertical position) for future use
    _hist_position_estimate_z.push_back(_position_base.z);
	_hist_gps_position_estimate_z.push_back(_position_base.z);

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    _historic_xy_counter++;
    if( _historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS ) {
        _historic_xy_counter = 0;
        _hist_position_estimate_x.push_back(_position_base.x);
        _hist_position_estimate_y.push_back(_position_base.y);
    }
}

//
// XY Axis specific methods
//

// position_ok - return true if position has been initialised and have received gps data within 3 seconds
bool AP_InertialNav::position_ok() const
{
    return _xy_enabled;
}

// check_gps - check if new gps readings have arrived and use them to correct position estimates
void AP_InertialNav::check_gps()
{
    const uint32_t now = hal.scheduler->millis();
    bool flag_RTK_FIXED = false;
    // compare gps time to previous reading
    const AP_GPS &gps = _ahrs.get_gps();
    if(gps.last_fix_time_ms() != _gps_last_time ) {
    	flag_RTK_FIXED = (gps.RTK_status() == AP_GPS::RTK_FIXED);
        // call position correction method
        correct_with_gps(now, gps.location().lng, gps.location().lat,gps.location().alt,flag_RTK_FIXED);

        // record gps time and system time of this update
        _gps_last_time = gps.last_fix_time_ms();
    }else{
        // if GPS updates stop arriving degrade position error to 10% over 2 seconds (assumes 100hz update rate)
        if (now - _gps_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS) {
            _position_error.x *= 0.9886f;
            _position_error.y *= 0.9886f;
           // _position_error.z *= 0.9886f;
            // increment error count
            if (_flags.ignore_error == 0 && _error_count < 255 && _xy_enabled) {
                _error_count++;
            }
        }
    }
}

// correct_with_gps - modifies accelerometer offsets using gps
void AP_InertialNav::correct_with_gps(uint32_t now, int32_t lon, int32_t lat,int32_t alt,bool flag_RTK_FIXED)
{
    float dt,x,y,z;
    float hist_position_base_x, hist_position_base_y,hist_position_base_z;
    static uint8_t first_reads = 0;

    // calculate time since last gps reading
    dt = (float)(now - _gps_last_update) * 0.001f;

    // update last gps update time
    _gps_last_update = now;

    // discard samples where dt is too large
    if( dt > 1.0f || dt == 0.0f || !_xy_enabled) {
        return;
    }

    // calculate distance from base location
    x = (float)(lat - _ahrs.get_home().lat) * LATLON_TO_CM;
    y = (float)(lon - _ahrs.get_home().lng) * _lon_to_cm_scaling;
    z = (float)(alt - _ahrs.get_home().alt);

	hal.uartC->printf("x:%f,y:%f,z:%f\n",x,y,z);
	hal.uartC->printf("posx:%f,posy:%f,posz:%f\n",_position.x,_position.y,_position.z);

    // sanity check the gps position.  Relies on the main code calling GPS_Glitch::check_position() immediatley after a GPS update
    if (_glitch_detector.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 5hz update rate)
        _position_error.x *= 0.7943f;
        _position_error.y *= 0.7943f;
        _position_error.z *= 0.7943f;
    }else{
        // if our internal glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav position and velocity to gps values
        if (_flags.gps_glitching) {
            set_position_xy(x,y);
            _position_error.x = 0.0f;
            _position_error.y = 0.0f;
        }else{
            // ublox gps positions are delayed by 400ms
            // we store historical position at 10hz so 4 iterations ago
            if( _hist_position_estimate_x.is_full()) {
                hist_position_base_x = _hist_position_estimate_x.front();
                hist_position_base_y = _hist_position_estimate_y.front();
                hist_position_base_z = _hist_gps_position_estimate_z.front();
            }else{
                hist_position_base_x = _position_base.x;
                hist_position_base_y = _position_base.y;
                hist_position_base_z = _position_base.z;
            }

            // calculate error in position from gps with our historical estimate
            _position_error.x = x - (hist_position_base_x + _position_correction.x);
            _position_error.y = y - (hist_position_base_y + _position_correction.y);
            if(flag_RTK_FIXED && false){
			// discard first 10 reads but perform some initialisation
			
			if( first_reads <= 10 ) {
				set_altitude(z);
				first_reads++;
			}
            _position_error.z = z - (hist_position_base_z + _position_correction.z);
            }
        }
    }

    // update our internal record of glitching flag so that we can notice a change
    _flags.gps_glitching = _glitch_detector.glitching();
}

// get accel based latitude
int32_t AP_InertialNav::get_latitude() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return _ahrs.get_home().lat + (int32_t)(_position.x/LATLON_TO_CM);
}

// get accel based longitude
int32_t AP_InertialNav::get_longitude() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return _ahrs.get_home().lng + (int32_t)(_position.y / _lon_to_cm_scaling);
}

// setup_home_position - reset state for home position change
void AP_InertialNav::setup_home_position(void)
{
    // set longitude to meters scaling to offset the shrinking longitude as we go towards the poles
    _lon_to_cm_scaling = longitude_scale(_ahrs.get_home()) * LATLON_TO_CM;

    // reset corrections to base position to zero
    _position_base.x = 0.0f;
    _position_base.y = 0.0f;
    _position_base.z = 0.0f;
    _position_correction.x = 0.0f;
    _position_correction.y = 0.0f;
    _position_correction.z = 0.0f;
    _position.x = 0.0f;
    _position.y = 0.0f;
    _position.z = 0.0f;
    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();
    _hist_gps_position_estimate_z.clear();
    // set xy as enabled
    _xy_enabled = true;
}

// get accel based latitude
float AP_InertialNav::get_latitude_diff() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0;
    }

    return (_position.x/LATLON_TO_CM);
}

// get accel based longitude
float AP_InertialNav::get_longitude_diff() const
{
    // make sure we've been initialised
    if( !_xy_enabled ) {
        return 0.0f;
    }

    return (_position.y / _lon_to_cm_scaling);
}

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
void AP_InertialNav::set_velocity_xy(float x, float y)
{
    _velocity.x = x;
    _velocity.y = y;
}

// set_velocity_xy - set velocity in latitude & longitude directions (in cm/s)
float AP_InertialNav::get_velocity_xy() const
{
	return pythagorous2(_velocity.x, _velocity.y);
}

//
// Z Axis methods
//

// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void AP_InertialNav::check_baro()
{
    uint32_t baro_update_time;

    // calculate time since last baro reading (in ms)
    baro_update_time = _baro.get_last_update();
    if( baro_update_time != _baro_last_update && !rng_valid) {
        const float dt = (float)(baro_update_time - _baro_last_update) * 0.001f; // in seconds
        // call correction method
        correct_with_baro(_baro.get_altitude()*100.0f, dt);
        _baro_last_update = baro_update_time;
    }
}


// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void AP_InertialNav::correct_with_baro(float baro_alt, float dt)
{
    static uint8_t first_reads = 0;

    // discard samples where dt is too large
    if( dt > 0.5f ) {
        return;
    }

    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(baro_alt);
        first_reads++;
    }

    // sanity check the baro position.  Relies on the main code calling Baro_Glitch::check_alt() immediatley after a baro update
    if (_baro_glitch.glitching()) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
        _position_error.z *= 0.89715f;
    }else{
        // if our internal baro glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav alt to baro alt
        if (_flags.baro_glitching) {
            set_altitude(baro_alt);
            _position_error.z = 0.0f;
        }else{
            // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
            // so we should calculate error using historical estimates
            float hist_position_base_z;
            if (_hist_position_estimate_z.is_full()) {
                hist_position_base_z = _hist_position_estimate_z.front();
            } else {
                hist_position_base_z = _position_base.z;
            }

            // calculate error in position from baro with our estimate
            _position_error.z = baro_alt - (hist_position_base_z + _position_correction.z);
        }
    }

    // update our internal record of glitching flag so that we can notice a change
    _flags.baro_glitching = _baro_glitch.glitching();
}

// set_altitude - set base altitude estimate in cm
void AP_InertialNav::set_altitude( float new_altitude)
{
    _position_base.z = new_altitude;
    _position_correction.z = 0;
    _position.z = new_altitude; // _position = _position_base + _position_correction
    _hist_position_estimate_z.clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
    _hist_gps_position_estimate_z.clear(); // reset z history to avoid fake z velocity at next baro calibration (next rearm)
}

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void AP_InertialNav::update_gains()
{
    // X & Y axis time constant
    if (_time_constant_xy == 0.0f) {
        _k1_xy = _k2_xy = _k3_xy = 0.0f;
    }else{
        _k1_xy = 3.0f / _time_constant_xy;
        _k2_xy = 3.0f / (_time_constant_xy*_time_constant_xy);
        _k3_xy = 1.0f / (_time_constant_xy*_time_constant_xy*_time_constant_xy);
    }

    // Z axis time constant
    if (_time_constant_z == 0.0f) {
        _k1_z = _k2_z = _k3_z = 0.0f;
    }else{
    	_time_constant_z = 3.0;
        _k1_z = 3.0f / _time_constant_z;
        _k2_z = 3.0f / (_time_constant_z*_time_constant_z);
        _k3_z = 1.0f / (_time_constant_z*_time_constant_z*_time_constant_z);
    }
}

// set_velocity_z - get latest climb rate (in cm/s)
void AP_InertialNav::set_velocity_z(float z )
{
    _velocity.z = z;
}

// set_position_xy - sets inertial navigation position to given xy coordinates from home
void AP_InertialNav::set_position_xy(float x, float y)
{
    // reset position from home
    _position_base.x = x;
    _position_base.y = y;
    _position_correction.x = 0.0f;
    _position_correction.y = 0.0f;

    // clear historic estimates
    _hist_position_estimate_x.clear();
    _hist_position_estimate_y.clear();

    // add new position for future use
    _historic_xy_counter = 0;
    _hist_position_estimate_x.push_back(_position_base.x);
    _hist_position_estimate_y.push_back(_position_base.y);
}


void AP_InertialNav::check_rng_glitch()
{
    uint32_t now = hal.scheduler->millis(); // current system time
    float sane_dt;                  // time since last sane baro reading
    float accel_based_distance;     // movement based on max acceleration
    int32_t alt_projected;          // altitude estimate projected from previous iteration
    int32_t distance_cm;            // distance from baro alt to current alt estimate in cm
    bool all_ok;                    // true if the new baro alt passes sanity checks

    // baro sanity check variables
    static uint32_t    _last_good_update;  // system time of last baro update that passed checks
    static int32_t     _last_good_alt;     // last good altitude reported by the baro
    static float       _last_good_vel;     // last good velocity reported by the baro in cm/s

    // if not initialised or disabled update last good alt and exit
    if (!_flags_rng_initialised) {
        _last_good_update = now;
        _last_good_alt = _sonar.distance_cm();
        _last_good_vel = get_rng_climb_rate();
        _flags_rng_initialised = true;
        _flags_rng_glitching = false;
        return;
    }

    // calculate time since last sane baro reading in ms
    sane_dt = (now - _last_good_update) / 1000.0f;

    // estimate our alt from last known alt and velocity
    alt_projected = _last_good_alt + (_last_good_vel * sane_dt);

    // calculate distance from recent baro alt to current estimate
    int32_t rng_alt = (int)(_rng_alt);
    int32_t rng_vel = (int)(_rng_alt_vel);
    // calculte distance from projected distance
    distance_cm = labs(alt_projected - rng_alt);

    // all ok if within a given hardcoded radius
    if (distance_cm <= BARO_GLITCH_DISTANCE_OK_CM) {
        all_ok = true;
    }else{
        // or if within the maximum distance we could have moved based on our acceleration
        accel_based_distance = 0.5f * BARO_GLITCH_ACCEL_MAX_CMSS * sane_dt * sane_dt;
        all_ok = (distance_cm <= accel_based_distance);
    }

    // store updates to baro position
    if (all_ok) {
        // position is acceptable
        _last_good_update = now;
        _last_good_alt = rng_alt;
        _last_good_vel = get_rng_climb_rate();
    }

    // update glitching flag
    _flags_rng_glitching = !all_ok;
	hal.console->printf_P(PSTR("_flags_rng_glitching: %d\n"),_flags_rng_glitching);

}

void AP_InertialNav::correct_with_rng(float rng_alt, float dt)
{
    static uint8_t first_reads = 0;

	

    // discard samples where dt is too large
    if( dt > 0.5f ) {
        return;
    }
    // discard first 10 reads but perform some initialisation
    if( first_reads <= 10 ) {
        set_altitude(rng_alt);
        first_reads++;
    }


    // sanity check the baro position.  Relies on the main code calling Baro_Glitch::check_alt() immediatley after a baro update
    if (_flags_rng_glitching) {
        // failed sanity check so degrate position_error to 10% over 2 seconds (assumes 10hz update rate)
        _position_error.z *= 0.8f;
        // Debug("_flags_rng_glitching\n");
    }else{
        // if our internal baro glitching flag (from previous iteration) is true we have just recovered from a glitch
        // reset the inertial nav alt to baro alt
        if (_flags_rng_glitching_internal) {

            _position_error.z = 0.0f;
        }else{
            // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
            // so we should calculate error using historical estimates
            float hist_position_base_z;
            if (_hist_position_estimate_z.is_full()) {
                hist_position_base_z = _hist_position_estimate_z.front();
            } else {
                hist_position_base_z = _position_base.z;
            }

            // calculate error in position from baro with our estimate
            _position_error.z = rng_alt * _ahrs.cos_roll() * _ahrs.cos_pitch() - (hist_position_base_z + _position_correction.z);
			//hal.console->printf_P(PSTR("rng_alt %f\n"),(float)rng_alt);
			hal.console->printf_P(PSTR("_ahrs.cos_roll %f    _ahrs.cos_roll   %f\n"),_ahrs.cos_roll(),_ahrs.cos_pitch());
			baroHgtOffset = 0.1f * (_baro.get_altitude() *100.0f + _position.z) + 0.9f * baroHgtOffset;
        }
    }

    // update our internal record of glitching flag so that we can notice a change
    _flags_rng_glitching_internal = _flags_rng_glitching;
}
// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_InertialNav::get_rng_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware

     return _climb_rate_filter.slope() * 1.0e3f;
}
void AP_InertialNav::rng_inav_check()
{
    static uint32_t _rng_last_update;
    const uint32_t now = hal.scheduler->millis();
	readRangeFinder();

    if(_rng_last_update != _sonar.get_rng_update_time() && newDataRng) {
        const float dt = (float)(_sonar.get_rng_update_time() - _rng_last_update) * 0.001f; // in seconds
        _rng_last_update = _sonar.get_rng_update_time();
        // call correction method
        _climb_rate_filter.update((float)_sonar.distance_cm(), _rng_last_update);
        check_rng_glitch();
        correct_with_rng(_rng_alt, dt);
		rng_valid = true;
		
    }
    else
    {
    	/*if (now - _rng_last_update > AP_INTERTIALNAV_GPS_TIMEOUT_MS) {
            _position_error.z *= 0.8f;
			rng_valid = true;
            // Debug("RNG data updates stop arriving\n");
        }*/
		if (now - _rng_last_update > AP_INTERTIALNAV_RNG_TIMEOUT_MS)
			{
				rng_valid = false;
			}
    }
}
void AP_InertialNav::readRangeFinder(void)
{
   static float storedRngMeas[3];
   static float storedRngVel[3];
   static uint32_t storedRngMeasTime_ms[3];
   static uint32_t lastRngMeasTime_ms = 0;
   static uint8_t rngMeasIndex = 0;
   
   uint8_t midIndex;
   uint8_t maxIndex;
   uint8_t minIndex;
   // get theoretical correct range when the vehicle is on the ground
   rngOnGnd = 10;//_rng.ground_clearance_cm() * 0.01f;
   if (_sonar.healthy()== true && (last_update_time - lastRngMeasTime_ms) > 50) {
	   // store samples and sample time into a ring buffer
	   rngMeasIndex ++;
	   if (rngMeasIndex > 2) {
		   rngMeasIndex = 0;
	   }
	   storedRngMeasTime_ms[rngMeasIndex] = last_update_time;
	   storedRngMeas[rngMeasIndex] = (float)_sonar.distance_cm();
	   storedRngVel[rngMeasIndex] = (float)_sonar.radar_vel();
	   // check for three fresh samples and take median
	   bool sampleFresh[3];
	   for (uint8_t index = 0; index <= 2; index++) {
		   sampleFresh[index] = (last_update_time - storedRngMeasTime_ms[index]) < 500;
	   }
	   if (sampleFresh[0] && sampleFresh[1] && sampleFresh[2]) {
		   if (storedRngMeas[0] > storedRngMeas[1]) {
			   minIndex = 1;
			   maxIndex = 0;
		   } else {
			   maxIndex = 0;
			   minIndex = 1;
		   }
		   if (storedRngMeas[2] > storedRngMeas[maxIndex]) {
			   midIndex = maxIndex;
		   } else if (storedRngMeas[2] < storedRngMeas[minIndex]) {
			   midIndex = minIndex;
		   } else {
			   midIndex = 2;
		   }


		   if (storedRngVel[0] > storedRngVel[1]) {
			   minIndex = 1;
			   maxIndex = 0;
		   } else {
			   maxIndex = 0;
			   minIndex = 1;
		   }
		   if (storedRngVel[2] > storedRngVel[maxIndex]) {
			   midIndex = maxIndex;
		   } else if (storedRngVel[2] < storedRngVel[minIndex]) {
			   midIndex = minIndex;
		   } else {
			   midIndex = 2;
		   }
		   _rng_alt= max(storedRngMeas[midIndex],rngOnGnd)* _ahrs.cos_roll() * _ahrs.cos_pitch() ;
		   _rng_alt_vel = storedRngVel[midIndex] * _ahrs.cos_roll() * _ahrs.cos_pitch() ;
		   newDataRng = true;
		   rngValidMeaTime_ms = last_update_time;
		   hal.console->printf_P(PSTR("rng_alt %f\t"),(float)_rng_alt);
		   hal.console->printf_P(PSTR("rng_alt_vel %f\n"),(float)_rng_alt_vel);
		   hal.uartE->printf("");
		   // recall vehicle states at mid sample time for range finder
		   //RecallStates(statesAtRngTime, storedRngMeasTime_ms[midIndex] - 25);
	   }
	   else if (false) { //!vehicleArmed
	   
		   // if not armed and no return, we assume on ground range
		   _rng_alt = rngOnGnd;
		   _rng_alt_vel = 0;
		   newDataRng = true;
		   rngValidMeaTime_ms = last_update_time;
		   // assume synthetic measurement is at current time (no delay)
		   //statesAtRngTime = state;
	   } else {
		   newDataRng = false;
	   }
	   lastRngMeasTime_ms =  last_update_time;
   }

}

