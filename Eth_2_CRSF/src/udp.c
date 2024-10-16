#include "stm32f1xx_hal.h"
#include "udp.h"
#include "socket.h"

/*================================================*/
uint16_t lwip_htons(uint16_t n)
{
  return (uint16_t)PP_HTONS(n);
}
/*================================================*/
int32_t udp_send_packet(char sn, uint8_t * dst_ip, uint16_t dst_port, uint8_t * txbuf, uint16_t len)
{
	struct udp_hdr *udphdr;
	udphdr= (struct udp_hdr *)txbuf;
	udphdr->src = lwip_htons(dst_port);
	udphdr->dest = lwip_htons(dst_port);
	udphdr->len= lwip_htons(len + 8);
	udphdr->chksum = 0x0000;
	return sendto(sn, txbuf, len + 8, dst_ip, dst_port);
}
/*================================================*/