#include "w5500.h"
#include "wizchip_conf.h"
#include "stm32f103_board.h"
#include "socket.h"
#include "crsf.h"
/*=============================================================*/
void SystemClock_Config(void);
void BSP_GPIO_init(void);
void w5500_init(void);
void SPIx_init(void);

static int8_t udp_socket;
static uint8_t dst_ip[4] = {192, 168, 144, 20};
//static uint8_t dst_ip[4] = {192, 168, 1, 230};
extern TIM_HandleTypeDef TimHandle;
extern uint32_t tm_cnt;
UART_HandleTypeDef UartHandle;
uint8_t uart_rx_buf[DATA_BUF_SIZE];
uint32_t	uart_rx_num= 0;
static int recv_udp_packet(int socket, uint8_t *packet, uint32_t len, uint32_t *ip, uint16_t *port);
/*=============================================================*/
int main(void)
{
	uint32_t i, j;		
//	uint8_t  link_status;
	uint8_t  sck_state;
//	HAL_StatusTypeDef rx_state;	
	uint8_t  eth_rx_buf[DATA_BUF_SIZE];
	uint32_t from_ip;
	uint16_t from_port;
	int      rx_len;
	int      rx_tmout;	

//	wiz_PhyConf phy;
	rx_tmout= 0;
	HAL_Init();
	SystemClock_Config();
	BSP_GPIO_init();
	UART_init();		
	udp_socket= 0;	
	SPIx_init();
	w5500_init();
	if( (udp_socket = socket(0, Sn_MR_UDP, UDP_PORT, 0)) != 0)
			on_error();
	for(i=0; i < 1000; i++)
	{
		getsockopt(udp_socket , SO_STATUS, &sck_state);
		if(sck_state == SOCK_UDP)
			break;
	}			
	tm_init();
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
	while(1)
	{
		if(tm_cnt > 500)
		{
			tm_cnt= 0;
			BSP_LED_Toggle();
			rx_tmout++;
			if(rx_tmout > 20)
				BSP_LedTestOff();
		}		
#if 0				
		ctlwizchip(CW_GET_PHYLINK, (void*)&link_status);
		if(link_status != PHY_LINK_ON)
		{
			BSP_LedTestOff();
			continue;
		}
		getsockopt(udp_socket , SO_STATUS, &sck_state);
		if(sck_state != SOCK_UDP)
		{
			BSP_LedTestOff();
			continue;	
		}
#endif			
		rx_len = recv_udp_packet(udp_socket, eth_rx_buf, DATA_BUF_SIZE, &from_ip, &from_port);
		if(rx_len > 0) 	
		{
			BSP_LedTestOn();
			rx_tmout=0;
		}
//		wdt_refresh();
		if(uart_rx_num)
		{
			j= uart_rx_num;
			uart_rx_num= 0;		
			sendto(udp_socket, (uint8_t *) uart_rx_buf, j, dst_ip, UDP_DEST_PORT);			
#ifdef CRSF_DECODE				
			if(j == 26)
			{
				for (i=0, j= 2, bits = bitsavailable = 0; i < CRSF_MAX_CHANNEL; i++)
				{
					v= uart_rx_buf[3+i] &0x00ff;
					bits |= (v << bitsavailable);
					bitsavailable+= 8;
					if (bitsavailable >= CROSSFIRE_CH_BITS)
					{
						channels[j++]= bits & 0x7ff;
						bits >>= CROSSFIRE_CH_BITS;
						bitsavailable -= CROSSFIRE_CH_BITS;
					}	
				}				
				channels[0]= (uart_rx_buf[0]<< 8) | uart_rx_buf[1];
				channels[1]= uart_rx_buf[2];				
				sendto(udp_socket, (uint8_t *) channels, 36, dst_ip, UDP_DEST_PORT);			
			}
#endif
		}						
	}		
}
/*=============================================================*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    on_error();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    on_error();
}
/*=============================================================*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
/*=============================================================*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/*=============================================================*/
static uint32_t ntohl(uint32_t netlong)
{
#ifdef SYSTEM_LITTLE_ENDIAN
	return swapl(netlong);
#else
	return netlong;
#endif		
}
/*=============================================================*/
static int recv_udp_packet(int socket, uint8_t *packet, uint32_t len, uint32_t *ip, uint16_t *port)
{
	int ret;
	uint8_t sck_state;
	int16_t recv_len;

	ret = getsockopt(socket, SO_STATUS, &sck_state);
	if(ret != SOCK_OK) 
		return -1;

	if(sck_state == SOCK_UDP) 
	{
		ret = getsockopt(socket, SO_RECVBUF, &recv_len);
		if(ret != SOCK_OK) 
			return -1;
		if(recv_len) 
		{
			recv_len = recvfrom(socket, packet, len, (uint8_t *)ip, port);
			if(recv_len < 0) 
				return -1;
			*ip = ntohl(*ip);

			return recv_len;
		}
	}
	return -1;
}
/*=============================================================*/
