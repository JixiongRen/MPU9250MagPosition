#ifndef FRAME_PROTOCOL_H
#define FRAME_PROTOCOL_H

#include <stdint.h>

/**
 * @brief Calculate CRC16-CCITT-FALSE
 * @param data Pointer to data buffer
 * @param length Length of data
 * @return CRC16 value
 */
uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t length);

/**
 * @brief Build frame with header, timestamp, length and CRC
 * @param payload Pointer to payload data
 * @param payload_len Length of payload
 * @param frame_out Output frame buffer (must be at least payload_len + 12 bytes)
 * @return Total frame length
 */
uint16_t build_frame(const uint8_t *payload, uint16_t payload_len, uint8_t *frame_out);

#endif // FRAME_PROTOCOL_H
