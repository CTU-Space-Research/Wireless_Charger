#include "qi_transmitter.h"
//#include "main.c"

void modulateFskByte(bool modulated_FSK_bits[],unsigned char data_byte){
		int num_ones = 0; // counter for number of ones in input_byte
		uint16_t byte = 0;
		// start bit 
		modulated_FSK_bits[0]=0;
	  // data bits 
    for (int i = 0; i < 8; i++) {
        modulated_FSK_bits[i+1] = (data_byte >> i) & 0x01; // shift bits and store LSB first
        if (modulated_FSK_bits[i+1]) {
            num_ones++; // increment number of ones if bit is set
        }
    }
		// parity bit 
		modulated_FSK_bits[9] = (num_ones % 2 == 1); // calculate parity bit
		// stop bit 
		modulated_FSK_bits[10]=1;
		for (int i = 0; i < 11; i++) {
			byte |= (modulated_FSK_bits[i] ? 1 : 0) << (10 - i);
		}
}
uint8_t calculateCheckSum(DataPacketFSK packet){
	uint8_t check_sum = 0;
	uint8_t byte = 0;
	for (int i = 0; i < 8; i++) {
		byte |= packet.header[i+1] << i;
	}
	check_sum+=byte;
	byte=0;
	for (int j = 0; j < packet.message_length; j++){
		for (int i = 0; i < 8; i++) {
			byte |= packet.message[j][i+1] << i;
		}
		check_sum+=byte;
		byte=0;
	}
	return check_sum;
}

uint8_t demodulateAskByte(uint16_t encoded_byte,DecodedDataPacket *packet, uint8_t position) {
	// returns 0 if everything is ok
		// 1 if start bit isn't 0
		// 2 if parity bit does not match
		// 3 stop bit isn't 1
		// 4 if checksum doesn't match
    uint8_t ones; // counter for number of ones in the input byte
    uint8_t data_byte; // the 8-bit data byte to be returned
	int8_t signed_data_byte;
    // start bit 
    if (encoded_byte & 1) {
        // error: start bit should be 0 
        return 1;
    }
    // data bits 
	data_byte = (encoded_byte >> 1) & 0xFF; // shift to second bit and mask with 8 bits
    for (int i = 0; i < 8; i++) {
        if (data_byte & (1 << i)) {
            ones++;
        }
    }
    // parity bit 

    // stop bit 
    if (((encoded_byte >> 10) & 0x01) == 0) {
        // error: stop bit should be 1 
        return 3;
    }
	if (position==1){
	//header
		packet->header=data_byte;
		switch (data_byte){ 
			case 0x01: //SIG
				packet->message_length=1;
				break;
			case 0x03: //CE
				packet->message_length=1;
				break;
			//todo: dodelat ostatni pakety
		}
	}
	else if (position==3){ //message_length.... 
		//checksum
		uint8_t check_sum = packet->header;
		for (uint8_t j = 0; j < packet->message_length; j++){
			check_sum+= packet->message[j];
		}
		if (check_sum!=data_byte){
			return 4;				
		}
	} else{
		//message
		packet->message[(position-2)]=data_byte;
	}
	if (((ones % 2 == 0) != ((encoded_byte >> 9) & 0x01))) {    //error: parity bit does not match
        return 2;
    }
    return 0;
}