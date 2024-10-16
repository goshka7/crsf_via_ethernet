#ifndef UDP_H_INCLUDED
#define UDP_H_INCLUDED
#include "w5500.h"

#define UDP_TX_PACKET_MAX_SIZE 24
#define UDP_HLEN 8
#define PP_HTONS(x) ((((x) & 0x00ffUL) << 8) | (((x) & 0xff00UL) >> 8))

__packed struct udp_hdr 
{
  uint16_t src;
  uint16_t dest;  /* src/dest UDP ports */
  uint16_t len;
  uint16_t chksum;
};

int32_t udp_send_packet(char sn, uint8_t * dst_ip, uint16_t dst_port, uint8_t * txbuf, uint16_t len);


#endif
