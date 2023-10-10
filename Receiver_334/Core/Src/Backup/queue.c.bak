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

bool queue_enqueue(Queue *q, DataPacket item) {
    if (queue_is_full(q)) {
        return false;
    }

    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->size++;
    return true;
}

bool queue_dequeue(Queue *q, DataPacket *item) {
    if (queue_is_empty(q)) {
        return false;
    }

    *item = q->buffer[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->size--;
    return true;
}

uint16_t get_data_packet_size(DataPacket packet) {
    uint16_t size = 0;
    size += PREAMBLE_LENGTH;
    size += HEADER_LENGTH;
    size += packet.message_length * 11;
    size += CHECKSUM_LENGTH;
    return size;
}


void serialize_data_packet(DataPacket packet, bool* serialized_data) {
    uint16_t size = get_data_packet_size(packet); //nemusí bejt
    uint16_t index = 0;
    for (int i = 0; i < PREAMBLE_LENGTH; i++) {
        serialized_data[index++] = packet.preamble[i];
    }
    for (int i = 0; i < HEADER_LENGTH; i++) {
        serialized_data[index++] = packet.header[i];
    }
    for (int i = 0; i < packet.message_length; i++) {
        for (int j = 0; j < 11; j++) {
            serialized_data[index++] = packet.message[i][j];
        }
    }
    for (int i = 0; i < CHECKSUM_LENGTH; i++) {
        serialized_data[index++] = packet.checksum[i];
    }
}