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

#include "a7105.h"
#include "lib_spi.h"

void A7105_WriteID(uint32_t ida) {
    lib_spi_ss_on();
    lib_spi_xfer(A7105_06_ID_DATA); //ex id=0x5475c52a ;txid3txid2txid1txid0
    lib_spi_xfer((ida >> 24)&0xff); //53 
    lib_spi_xfer((ida >> 16)&0xff); //75
    lib_spi_xfer((ida >> 8)&0xff); //c5
    lib_spi_xfer((ida >> 0)&0xff); //2a
    lib_spi_ss_off();
}

// read 4 bytes ID

void A7105_ReadID(uint8_t *_aid) {
    uint8_t i;
    lib_spi_ss_on();
    lib_spi_xfer(0x46);
    for (i = 0; i < 4; i++) {
        _aid[i] = lib_spi_xfer(0x00);
    }
    lib_spi_ss_off();
}

void A7105_WritePayload(uint8_t *_packet, uint8_t len) {
    uint8_t i;
    lib_spi_ss_on();
    lib_spi_xfer(A7105_RST_WRPTR);
    lib_spi_xfer(0x05);
    for (i = 0; i < len; i++) {
        lib_spi_xfer(_packet[i]);
    }
    lib_spi_ss_off();
}

void A7105_ReadPayload(uint8_t *_packet, uint8_t len) {
    uint8_t i;
    lib_spi_ss_on();
    lib_spi_xfer(0x45); //cmd read rx packet
    for (i = 0; i < len; i++) {
        _packet[i] = lib_spi_xfer(0x00);
    }
    lib_spi_ss_off();
}

void A7105_Reset(void) {
    A7105_WriteRegister(A7105_00_MODE, 0x00);
}

uint8_t A7105_ReadRegister(uint8_t address) {
    uint8_t result;
    lib_spi_ss_on();
    address |= 0x40; // set R/W bit
    lib_spi_xfer(address);
    result = lib_spi_xfer(0x00);
    lib_spi_ss_off();
    return (result);
}

void A7105_WriteRegister(uint8_t address, uint8_t data) {
    lib_spi_ss_on();
    lib_spi_xfer(address & 0xBF); //clear R/W bit
    lib_spi_xfer(data);
    lib_spi_ss_off();
}

void A7105_WriteRaw(int * data, uint8_t len){
    lib_spi_ss_on();
    for (int i = 0; i < len; i++) {
        lib_spi_xfer(data[i]);
    }
    lib_spi_ss_off(); 
}

void A7105_Strobe(uint8_t command) {
    lib_spi_ss_on();
    lib_spi_xfer(command);
    lib_spi_ss_off();
}


