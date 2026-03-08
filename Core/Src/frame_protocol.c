#include "frame_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    const uint16_t poly = 0x1021;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

uint16_t build_frame(const uint8_t *payload, uint16_t payload_len, uint8_t *frame_out) {
    uint16_t idx = 0;
    
    // Frame header: AA 55 AA 55
    frame_out[idx++] = 0xAA;
    frame_out[idx++] = 0x55;
    frame_out[idx++] = 0xAA;
    frame_out[idx++] = 0x55;
    
    // Payload length (2 bytes, little-endian)
    frame_out[idx++] = (uint8_t)(payload_len & 0xFF);
    frame_out[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);
    
    // Timestamp (4 bytes, little-endian)
    uint32_t timestamp = xTaskGetTickCount();
    frame_out[idx++] = (uint8_t)(timestamp & 0xFF);
    frame_out[idx++] = (uint8_t)((timestamp >> 8) & 0xFF);
    frame_out[idx++] = (uint8_t)((timestamp >> 16) & 0xFF);
    frame_out[idx++] = (uint8_t)((timestamp >> 24) & 0xFF);
    
    // Payload data
    memcpy(&frame_out[idx], payload, payload_len);
    idx += payload_len;
    
    // CRC16 (2 bytes, little-endian)
    uint16_t crc = crc16_ccitt_false(frame_out, idx);
    frame_out[idx++] = (uint8_t)(crc & 0xFF);
    frame_out[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    
    return idx;
}
