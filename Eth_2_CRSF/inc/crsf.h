#ifndef UDP_CRSF_RX_H
#define UDP_CRSF_RX_H
#include "stm32f1xx_hal.h"


// from https://github.com/DeviationTX/deviation/pull/1009/ ELRS menu implement in deviation TX
/*static uint8_t  currentPktRate =1; //  "250Hz", "150Hz", "50Hz"
                                      1        3       5
static uint8_t  currentPower =1 ;//  "10mW", "25mW", "50mW", "100mW", "250mW"
                                   0     1         2        3        4
*/
#define CROSSFIRE_CH_BITS           11
#define CRSF_CRC_POLY 0xd5

#define CRSF_MAX_CHANNEL 16
#define TYPE_CHANNELS           		0x16
#define CRSF_PAYLOAD_SIZE_MAX           60
#define CRSF_PACKET_LENGTH              22
#define CRSF_PACKET_SIZE                26
#define CRSF_FRAME_LENGTH               24 // length of type + payload + crc
#define CRSF_CMD_PACKET_SIZE            8

#define CRSF_SYNC_BYTE 0xC8

// ELRS command
#define ELRS_ADDRESS                    0xEE
#define ELRS_BIND_COMMAND               0xFF
#define ELRS_WIFI_COMMAND               0xFE
#define ELRS_PKT_RATE_COMMAND           0x01
#define ELRS_TLM_RATIO_COMMAND          0x02
#define ELRS_SWITCH_MODE_COMMAND        0x03
#define ELRS_MODEL_MATCH_COMMAND        0x04
#define ELRS_POWER_COMMAND              0x06
#define ELRS_BLE_JOYSTIC_COMMAND        17
#define TYPE_SETTINGS_WRITE             0x2D
#define ADDR_RADIO                      0xEA //  Radio Transmitter

void make_crsf_packet(uint8_t *packet, uint8_t addr, int16_t *channels);
void make_crsfCmd_packet(uint8_t *packet, uint8_t command, uint8_t value);
uint8_t crsf_crc8( uint8_t *ptr, uint8_t len) ;
void crc8_init(uint8_t poly);

#endif // UDP_CRSF_RX_H
