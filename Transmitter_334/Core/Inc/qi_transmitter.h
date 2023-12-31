//#ifndef _qi_transmitter_h
//#define _qi_transmitter_h

#include "stdbool.h"
#include "queue.h"
typedef enum {D_NULL=0x00,D_SIG=0x01,D_EPT=0x02,D_CE=0x03,D_NEGO=0x09,D_DSR=0x15,D_SRQ=0x20,D_ADC=0x25,D_RP=0x31}DATA_PACKETS; // pridat je�te ADT a PROP, maj� ruzne oznaceni podle velikosti


void modulateFskByte(bool modulatedFSK_bits[],unsigned char data_byte);
uint8_t demodulateAskByte(uint16_t encoded_byte,DecodedDataPacket *packet, uint8_t position);
uint8_t calculateCheckSum(DataPacketFSK packet);