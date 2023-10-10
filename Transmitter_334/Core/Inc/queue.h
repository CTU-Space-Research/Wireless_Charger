#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#define QUEUE_SIZE 3

#define MAX_MESSAGE_LENGTH_ASK 2 //zatim max 4 byty
#define HEADER_LENGTH_FSK 11
#define MAX_MESSAGE_LENGTH_FSK 1
#define CHECKSUM_LENGTH_FSK 11
#define MAX_DATA_PACKET_SIZE_FSK (HEADER_LENGTH_FSK + MAX_MESSAGE_LENGTH_FSK * 11 + CHECKSUM_LENGTH_FSK)
typedef struct {
    bool header[HEADER_LENGTH_FSK];
    bool message[MAX_MESSAGE_LENGTH_FSK][11];
    uint8_t message_length;
    bool checksum[CHECKSUM_LENGTH_FSK];
} DataPacketFSK;

typedef struct {
    uint8_t header;
    uint8_t message[MAX_MESSAGE_LENGTH_ASK];
    uint8_t message_length;
} DecodedDataPacket;

typedef struct {
    DataPacketFSK buffer[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t size;
} Queue;

void queue_init(Queue *q);
bool queue_is_empty(Queue *q);
bool queue_is_full(Queue *q);
bool queue_enqueue(Queue *q, DataPacketFSK  item);
bool queue_dequeue(Queue *q, DataPacketFSK  *item);
uint16_t get_data_packet_size(DataPacketFSK  packet);
void serialize_data_packet(DataPacketFSK  packet, bool* serialized_data);
void serialize_response_pattern(uint8_t response_pattern, bool* serialized_data);
#endif