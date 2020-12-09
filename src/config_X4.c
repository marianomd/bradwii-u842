/* 
Copyright 2014 Goebish

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "bradwii.h"
#include "config_X4.h"

extern usersettingsstruct usersettings;

/* The Hubsan X4 does not have a serial port to connect it
   to a GUI such as MultiwiiConfig GUI, settings are configured
   from here until a better solution is found */
void x4_set_usersettings() {
    // set acro mode rotation rates
    usersettings.maxyawrate = 600L << FIXEDPOINTSHIFT; // degrees per second
    usersettings.maxpitchandrollrate = 400L << FIXEDPOINTSHIFT; // degrees per second

    // pitch PIDs
    usersettings.pid_pgain[PITCHINDEX] = 35L << 3;
    usersettings.pid_igain[PITCHINDEX] = 4L;
    usersettings.pid_dgain[PITCHINDEX] = 22L << 2;

    // roll PIDs
    usersettings.pid_pgain[ROLLINDEX] = 35L << 3;
    usersettings.pid_igain[ROLLINDEX] = 4L;
    usersettings.pid_dgain[ROLLINDEX] = 22L << 2;

    // yaw PIDs
    usersettings.pid_pgain[YAWINDEX] = 100L << 3;
    usersettings.pid_igain[YAWINDEX] = 0L;
    usersettings.pid_dgain[YAWINDEX] = 22L << 2;

    for (int x = 0; x < NUMPOSSIBLECHECKBOXES; ++x) {
        usersettings.checkboxconfiguration[x] = 0;
    }

    // flight modes, see checkboxes.h for a complete list
    usersettings.checkboxconfiguration[CHECKBOXARM] = CHECKBOXMASKAUX3HIGH;
    usersettings.checkboxconfiguration[CHECKBOXYAWHOLD] = CHECKBOXMASKAUX4HIGH;

    // set fullacro flight mode (gyro only) for  AUX1 high (LEDs on/off channel on stock TX)
    // default for H107L, H107C & H107D stock TXs
    //usersettings.checkboxconfiguration[CHECKBOXFULLACRO] = CHECKBOXMASKAUX1HIGH; // rate mode (gyro only)
    //usersettings.checkboxconfiguration[CHECKBOXHIGHRATES] = CHECKBOXMASKAUX1HIGH; // uncommentr for high rates

    // set semiacro flight mode for AUX1 low
    // default for H107 stock TX 
    //usersettings.checkboxconfiguration[CHECKBOXSEMIACRO] = CHECKBOXMASKAUX1LOW;
    //usersettings.checkboxconfiguration[CHECKBOXHIGHANGLE] = CHECKBOXMASKAUX1LOW; // uncomment for high angle
}

