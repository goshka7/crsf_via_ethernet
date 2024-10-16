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
extern TIM_HandleTypeDef TimHandle;
extern uint32_t tm_cnt;
UART_HandleTypeDef UartHandle;
uint32_t uart_rx_num= 0;
uint32_t crsf_rate_cnt= 0;
uint8_t uart_rx_buf[DATA_BUF_SIZE];
static uint8_t dst_ip[4] = {192, 168, 1, 110};
static int recv_udp_packet(int socket, uint8_t *packet, uint32_t len, uint32_t *ip, uint16_t *port);
__IO ITStatus UartReady;
__IO ITStatus HZ_100;
/*=============================================================*/
static void default_channels(int16_t *channels)
{
	int i;
	for(i=0; i < 16; i++)
		channels[i]= (i <= 3)? 0x3e0 : (i == 4)? 0x700 : 0xae;	
}
/*=============================================================*/	
int main(void)
{
	uint32_t i;		
//	uint8_t link_status;
	int8_t sck_state;
	uint32_t from_ip;
	uint16_t from_port;
	int rx_len;
	int rx_len_ok=0;
	int rx_tmout_cnt;
	int rx_cnt;
	int16_t channels[16];
	uint8_t eth_rx_buf[DATA_BUF_SIZE];	
	uint8_t crsf_packet[DATA_BUF_SIZE];	
	uint8_t uart_txbuf[DATA_BUF_SIZE];		
//	int16_t tx_cnt=0;
//	HAL_StatusTypeDef rx_state;	
//	wiz_PhyConf phy;
	rx_cnt= 0;
	HAL_Init();
	SystemClock_Config();
	BSP_GPIO_init();
	UART_init();	
	udp_socket= 0;
	rx_tmout_cnt= 0;
	SPIx_init();
	w5500_init();
	crc8_init(CRSF_CRC_POLY);
	if( (udp_socket = socket(0, Sn_MR_UDP, UDP_DEST_PORT, 0)) != 0)
			on_error();
	default_channels( channels);
	make_crsf_packet(crsf_packet, CRSF_SYNC_BYTE, channels);	
	rx_len_ok= 26;	
	for(i=0; i < 1000; i++)
	{
		getsockopt(udp_socket , SO_STATUS, &sck_state);
		if(sck_state == SOCK_UDP)
			break;
	}			
	tm_init();	
	UartReady = SET;
	HZ_100= RESET;
	while(1)
	{
		if(tm_cnt > 25)
		{
			tm_cnt= 0;
			BSP_LED_Toggle();	
		}
		if(HZ_100 == SET) /*   50 hz */
		{
			HZ_100 = RESET;
			crsf_rate_cnt= 0;					
			for(i=0; i< 26; i++)
				uart_txbuf[i]= crsf_packet[i];
//			HAL_UART_Transmit_IT(&UartHandle, (uint8_t *) uart_txbuf, rx_len_ok);	
			if(UartReady == SET)
			{
				HAL_UART_Transmit_DMA(&UartHandle, (uint8_t *)uart_txbuf, rx_len_ok);
				UartReady = RESET;
			}
			rx_tmout_cnt++;
		}			
		if(rx_tmout_cnt > 1)
		{
			BSP_LedTestOff();				
//			make_crsf_packet(crsf_packet, CRSF_SYNC_BYTE, channels);			
//			rx_len_ok= 26;
		}
		i = getsockopt(udp_socket, SO_STATUS, &sck_state);
		if(i != SOCK_OK) 
			continue;
		if(sck_state == SOCK_CLOSED) 
		{
			if( (udp_socket = socket(0, Sn_MR_UDP, UDP_DEST_PORT, 0)) != 0)
				on_error();
			i = getsockopt(udp_socket, SO_STATUS, &sck_state);
			if(i != SOCK_OK) 
				continue;			
		}
		if(sck_state == SOCK_UDP) 
		{	
			rx_len = recv_udp_packet(udp_socket, (uint8_t *) eth_rx_buf, DATA_BUF_SIZE, &from_ip, &from_port);
			if(rx_len > 0) 
			{
				for(i=0; i< rx_len; i++)
					crsf_packet[i]= eth_rx_buf[i];
				rx_len_ok= rx_len;
				BSP_LedTestOn();
				rx_cnt++;
				if(rx_cnt > 50)
				{
					rx_cnt= 0;
					sendto(udp_socket, eth_rx_buf, rx_len, dst_ip, UDP_PORT);
				}
				rx_tmout_cnt= 0;
			}
		}
//		wdt_refresh();
#if 0				
				ctlwizchip(CW_GET_PHYLINK, (void*)&link_status);
				if(link_status != PHY_LINK_ON)
					continue;
				getsockopt(udp_socket , SO_STATUS, &sck_state);
				if(sck_state != SOCK_UDP)
					continue;	
#endif				
	}		
}
/*================================================================*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    on_error();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    on_error();
  }
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
uint32_t ntohl(uint32_t netlong)
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
	int16_t recv_len;
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
	return -1;
}

/*=============================================================*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  UartReady = SET; 
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    on_error();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
 on_error();

}
