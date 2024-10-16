#include "stm32f1xx_nucleo.h"
#include "w5500.h"
/*=============================================================*/
SPI_HandleTypeDef hspi1;
/*=============================================================*/
void W5500_Select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}
/*=============================================================*/
void W5500_Unselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}
/*=============================================================*/
void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}
/*=============================================================*/
void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}
/*=============================================================*/
uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}
/*=============================================================*/
void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}
/*=============================================================*/
void w5500_init(void) {
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
		uint8_t tmpstr[6];
    ctlwizchip(CW_GET_ID,(void*)tmpstr);
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
    wiz_NetInfo net_info = {
        .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
        .dhcp = NETINFO_STATIC,
				.ip   = {192.168.1.126},
				.sn   = {255.255.0.0},
				.gw   = {192.168.1.1}
    };
    setSHAR(net_info.mac);
    wizchip_setnetinfo(&net_info);
}

void loop() {
    HAL_Delay(1000);
}
