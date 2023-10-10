#include "queue.h"

void queue_init(Queue *q) {
    q->head = 0;
    q->tail = 0;
    q->size = 0;
}

bool queue_is_empty(Queue *q) {
    return q->size == 0;
}

bool queue_is_full(Queue *q) {
    return q->size == QUEUE_SIZE;
}

bool queue_enqueue(Queue *q, DataPacketFSK item) {
    if (queue_is_full(q)) {
        return false;
    }

    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->size++;
    return true;
}

bool queue_dequeue(Queue *q, DataPacketFSK *item) {
    if (queue_is_empty(q)) {
        return false;
    }

    *item = q->buffer[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->size--;
    return true;
}

uint16_t get_data_packet_size(DataPacketFSK packet) {
    uint16_t size = 0;
    size += HEADER_LENGTH_FSK;
    size += packet.message_length * 11;
    size += CHECKSUM_LENGTH_FSK;
    return size;
}
void serialize_response_pattern(uint8_t response_pattern, bool* serialized_data){
    uint16_t index = 0;
    for (int i = 0; i < 8; i++) {
        serialized_data[i] = (response_pattern >> i) & 0x01;
    }
}
void serialize_data_packet(DataPacketFSK  packet, bool* serialized_data){
    uint16_t index = 0;
    for (int i = 0; i < HEADER_LENGTH_FSK; i++) {
        serialized_data[index++] = packet.header[i];
    }
    for (int i = 0; i < packet.message_length; i++) {
        for (int j = 0; j < 11; j++) {
            serialized_data[index++] = packet.message[i][j];
        }
    }
    for (int i = 0; i < CHECKSUM_LENGTH_FSK; i++) {
        serialized_data[index++] = packet.checksum[i];
    }
}

