/*
 * rx_flysky.c
 *
 *  Created on: 11 juil. 2015
 *      Author: franck
 */


#include "bradwii.h"
#include "rx.h"
#include "a7105.h"
#include "lib_timers.h"
#include <stdint.h>
#include "config_X4.h"
#include "leds.h"
#include "eeprom.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

static const uint8_t tx_channels[16][16] = {
    { 0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
    { 0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
    { 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
    { 0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
    { 0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
    { 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
    { 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
    { 0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
    { 0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
    { 0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
    { 0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    { 0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
    { 0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
    { 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
    { 0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    { 0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};
static const uint8_t A7105_regs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x19, // <- 0x0b: 0x19 GIO1 4 wire MISO
    0x01, 0x05, 0x00, 0x50, 0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00,
    0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f, 0x13, 0xc3, 0x00, 0xff,
    0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f, 0xff,
};

static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static uint8_t channel;

extern globalstruct global;
extern usersettingsstruct usersettings;

#define BIND_READ_TOGGLE_TIMEOUT 1000000

static uint8_t packet[21];

void init_channels(void);
int _readrx(void);

void init_a7105(void) {
    uint8_t i;
    uint8_t if_calibration1;
    uint8_t vco_calibration0;
    uint8_t vco_calibration1;
    lib_timers_delaymilliseconds(10); //wait 10ms for A7105 wakeup
    A7105_Reset(); //reset A7105
    A7105_WriteID(0x5475c52A); // flysky protocol id

    for (i = 0; i < 0x33; i++) {
        if (A7105_regs[i] != 0xff)
            A7105_WriteRegister(i, A7105_regs[i]);

    }
    A7105_Strobe(A7105_STANDBY); //stand-by
    A7105_WriteRegister(0x02, 0x01);
    while (A7105_ReadRegister(0x02)) {
        if_calibration1 = A7105_ReadRegister(0x22);
        if (if_calibration1 & 0x10) { //do nothing
        }
    }

    A7105_WriteRegister(0x24, 0x13);
    A7105_WriteRegister(0x26, 0x3b);
    A7105_WriteRegister(0x0F, 0x00); //channel 0
    A7105_WriteRegister(0x02, 0x02);
    while (A7105_ReadRegister(0x02)) {
        vco_calibration0 = A7105_ReadRegister(0x25);
        if (vco_calibration0 & 0x08) { //do nothing
        }
    }

    A7105_WriteRegister(0x0F, 0xA0);
    A7105_WriteRegister(0x02, 0x02);
    while (A7105_ReadRegister(0x02)) {
        vco_calibration1 = A7105_ReadRegister(0x25);
        if (vco_calibration1 & 0x08) { //do nothing
        }
    }

    A7105_WriteRegister(0x25, 0x08);
    A7105_Strobe(A7105_STANDBY); //stand-by
}

void init_channels() {
    chanrow = usersettings.flysky_id % 16;
    chanoffset = (usersettings.flysky_id & 0xff) / 16;
    chancol = 0;
    //	Serial.print("chanoffset=");
    //	Serial.print(chanoffset, HEX);
    if (chanoffset > 9)
        chanoffset = 9; //from sloped soarer findings, bug in flysky protocol
    //initiallize default ppm values
    //	for (int i = 0; i < chanel_number; i++) {
    //		ppm[i] = default_servo_value;
    //	}
}

/*
fixedpointnum scaleValue(uint16_t value, uint16_t minInput, uint16_t maxInput, fixedpointnum minOutput, fixedpointnum maxOutput) {
        fixedpointnum temp=value;
        if (value<minInput) temp=minInput;
        if (value>maxInput) temp=maxInput;
        fixedpointnum scalefactor=(maxOutput-minOutput)/(maxInput-minInput);
        temp-=(maxInput+minInput)/2;
        return temp*scalefactor;
}
 */

uint32_t bind_Flysky() {
    unsigned long timeout_timer = lib_timers_starttimer();
    uint32_t _id = 0;
    // set channel 0;
    A7105_Strobe(0xA0);
    A7105_WriteRegister(A7105_0F_PLL_I, 00);
    A7105_Strobe(A7105_RX);
    while (!_id && lib_timers_gettimermicroseconds(timeout_timer) < BIND_READ_TOGGLE_TIMEOUT) {
        if (lib_timers_gettimermicroseconds(0) % 524288 > 262144)
            leds_set(LED2);
        else
            leds_set(LED1);

        char mode = A7105_ReadRegister(A7105_00_MODE);

        if (mode & A7105_MODE_TRER_MASK || mode & (1 << 6) || mode & (1 << 5)) {
            A7105_Strobe(A7105_RST_RDPTR);
            A7105_Strobe(A7105_RX);
            continue;
        }
        A7105_ReadPayload((uint8_t*) & packet, sizeof (packet));
        A7105_Strobe(A7105_RST_RDPTR);
        if (packet[0] == 170) // 170
        {
            //set found tx		
            _id = ((packet[1] << 0 | packet[2] << 8 | packet[3] << 16 | packet[4] << 24));
        }
        A7105_Strobe(A7105_RX);
    }
    return _id;
}

#define READ_STATE 1
#define BIND_STATE 2
#define DONE 0

void bind() {
    /* if no id found, bind */
    while (usersettings.flysky_id == 0) {
        usersettings.flysky_id = bind_Flysky();
    }
    writeusersettingstoeeprom();
    /* if id found toggle read and bind until one frame is readed */
    /* in that way, it possible to bind with an other TX even if one tx id is already registered */
    uint8_t state = READ_STATE;
    while (state != DONE) {
        unsigned long timeout_timer = lib_timers_starttimer();
        if (state == READ_STATE) {
            init_channels();
            if (_readrx()) state = DONE;
            while (state == READ_STATE && lib_timers_gettimermicroseconds(timeout_timer) < BIND_READ_TOGGLE_TIMEOUT) {
                if (lib_timers_gettimermicroseconds(0) % 500000 > 250000)
                    leds_set(LED1);
                else
                    leds_set(LED2);
                if (_readrx()) state = DONE;
            }
            if (state == READ_STATE) {
                state = BIND_STATE;
            }
        }
        if (state == BIND_STATE) {
            uint32_t _id = bind_Flysky();
            if (_id) {
                usersettings.flysky_id = _id;
            }
            state = READ_STATE;
        }
    }
}

uint32_t convert2id(uint8_t txid0, uint8_t txid1, uint8_t txid2, uint8_t txid3) {
    return txid0 | ((uint32_t) txid1 << 8) | ((uint32_t) txid2 << 16) | ((uint32_t) txid3 << 24);
}

#define PPM_OFFSET 1500

fixedpointnum scaleValue(fixedpointnum value, uint16_t deadband) {
    fixedpointnum newValue = (((fixedpointnum) value) - PPM_OFFSET)*131L;
    if (newValue < deadband && newValue>-deadband) return 0;
    return newValue;
}

void rx_fp_lowpassfilter(fixedpointnum *variable, fixedpointnum newvalue, fixedpointnum timesliver, fixedpointnum oneoverperiod, int timesliverextrashift) {
    //lib_fp_lowpassfilter(variable, newvalue, timesliver, oneoverperiod, timesliverextrashift);
    *variable = newvalue;
}

void decodepacket() {

    rx_fp_lowpassfilter(&global.rxvalues[ROLLINDEX],
            scaleValue(packet[5] + 256 * packet[6], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[PITCHINDEX],
            scaleValue(packet[7] + 256 * packet[8], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[THROTTLEINDEX],
            scaleValue(packet[9] + 256 * packet[10], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[YAWINDEX],
            scaleValue(packet[11] + 256 * packet[12], 1000)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[AUX1INDEX],
            scaleValue(packet[13] + 256 * packet[14], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[AUX2INDEX],
            scaleValue(packet[15] + 256 * packet[16], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[AUX3INDEX],
            scaleValue(packet[17] + 256 * packet[18], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
    rx_fp_lowpassfilter(&global.rxvalues[AUX4INDEX],
            scaleValue(packet[19] + 256 * packet[20], 0)
            /*((fixedpointnum) word_temp )*/, global.timesliver, 0L, 0);
}

int checkpacket() {
    if (packet[0] == 0x55) {// corrupt packet probably ( or bind packet)
        for (int i = 6; i <= 16; i = i + 2) {
            // upper bits of the channel msb's should be all 0 
            if (packet[i] & 0xF0) {// corrupt packet ( or strange transmitter protocol)
                return 0;
            }
        }
        return 1;
    }
    return 0;
}

int _readrx(void) {
    for (int i = 0; i < 21; i++) packet[i] = 0;
    channel = tx_channels[chanrow][chancol] - 1 - chanoffset;
    A7105_Strobe(A7105_STANDBY);
    A7105_Strobe(A7105_RST_RDPTR);
    A7105_WriteRegister(0x0F, channel);
    A7105_Strobe(A7105_RX);
    chancol = (chancol + 1) % 16;
    unsigned long pause;
    uint8_t x;
    pause = lib_timers_starttimer();
    while (1) {
        if (lib_timers_gettimermicroseconds(pause) > 2000) {
            //			Red_LED_OFF;
            chancol = (chancol + 1) % 16;
            channel = tx_channels[chanrow][chancol] - 1 - chanoffset;
            break;
        }
        if (A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK) {
            continue;
        }
        x = A7105_ReadRegister(0x00);
        if (!(bitRead(x, 5) == 0)&& !(bitRead(x, 6) == 0)) {
            continue;
        }
        A7105_ReadPayload((uint8_t*) & packet, 21);

        if (!checkpacket()) {
            // invalid packet which passed crc or bind packet
            A7105_Strobe(A7105_RX);
            return;
        }

        if (convert2id(packet[1], packet[2], packet[3], packet[4]) != usersettings.flysky_id) {
            //			Serial.println("bad id");
            continue;
        }

        decodepacket();

        // reset the failsafe timer
        global.failsafetimer = lib_timers_starttimer();
        return 1;
    }
    return 0;
}

void initrx(void) {
    init_a7105();
    bind();
}

void readrx(void) {
    _readrx();
}

