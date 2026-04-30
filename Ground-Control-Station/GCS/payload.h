/* payload.h */
#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>

/* Standard 32-Byte NRF24 Payload Limit - We use 12 bytes for hyper-efficiency */
typedef struct __attribute__((packed)) {
    uint8_t  magic;       /* Always 0x5A (90) to verify packet integrity */
    uint8_t  state;       /* 0 = Disarmed, 1 = Armed, 2 = Failsafe */
    int16_t  throttle;    /* 0 to 1000 */
    int16_t  pitch;       /* -1000 to 1000 */
    int16_t  roll;        /* -1000 to 1000 */
    int16_t  yaw;         /* -1000 to 1000 */
    uint16_t checksum;    /* Basic additive checksum */
} rc_payload_t; 

#endif
