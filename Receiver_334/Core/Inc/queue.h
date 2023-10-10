#ifndef QUEUE_H
#define QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#define QUEUE_SIZE 3
#define PREAMBLE_LENGTH 8
#define HEADER_LENGTH 11
#define MAX_MESSAGE_LENGTH 4 //zatim max 4 byty
#define CHECKSUM_LENGTH 11
#define MAX_DATA_PACKET_SIZE (PREAMBLE_LENGTH + HEADER_LENGTH + MAX_MESSAGE_LENGTH * 11 + CHECKSUM_LENGTH)
#define MAX_MESSAGE_LENGTH_FSK 1
typedef struct {
    bool preamble[PREAMBLE_LENGTH];
    bool header[HEADER_LENGTH];
    bool message[MAX_MESSAGE_LENGTH][11];
    uint8_t message_length;
    bool checksum[CHECKSUM_LENGTH];
} DataPacket;

typedef struct {
    uint8_t header;
    uint8_t message[MAX_MESSAGE_LENGTH_FSK];
    uint8_t message_length;
} DecodedDataPacket;

typedef struct {
    DataPacket buffer[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t size;
} Queue;

void queue_init(Queue *q);
bool queue_is_empty(Queue *q);
bool queue_is_full(Queue *q);
bool queue_enqueue(Queue *q, DataPacket item);
bool queue_dequeue(Queue *q, DataPacket *item);
uint16_t get_data_packet_size(DataPacket packet);
void serialize_data_packet(DataPacket packet, bool* serialized_data);
#endif