/*
 * nrf24l01p.c
 *
 * Created: 9/25/2015 1:53:43 PM
 *  Author: Nigil Lee
 */ 

#include "nrf24l01p.h"
#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include <util/delay.h>

// Chip SPI Commands
#define SPICMD_R_REGISTER(reg) (0x00 | reg)
#define SPICMD_W_REGISTER(reg) (0x20 | reg)
#define SPICMD_R_RX_PAYLOAD 0x61
#define SPICMD_W_TX_PAYLOAD 0xA0
#define SPICMD_FLUSH_TX 0xE1
#define SPICMD_FLUSH_RX 0xE2
#define SPICMD_REUSE_TX_PL 0xE3

// Register addresses
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_TX_ADDR 0x10

// CONFIG reg bits
#define CONFIG_PRIM_RX (1 << 0)
#define CONFIG_PWR_UP (1 << 1)

// RF SETUP reg bits
#define RF_SETUP_PLL_LOCK (1 << 4)
#define RF_SETUP_RF_DR (1 << 3)
#define RF_SETUP_RF_PWR0 (1 << 2)
#define RF_SETUP_RF_PWR1 (1 << 1)

// IO Definitions
#define NRF24L01P_MOSI_PORT PORTC
#define NRF24L01P_MOSI_PIN PIN5_bm

#define NRF24L01P_SCLK_PORT PORTC
#define NRF24L01P_SCLK_PIN PIN7_bm

#define NRF24L01P_CS_PORT PORTC
#define NRF24L01P_CS_PIN PIN4_bm

#define NRF24L01P_CE_PORT PORTD
#define NRF24L01P_CE_PIN PIN1_bm

#define NRF24L01P_SPI SPIC

// Static Function Prototypes
void static spi_startframe();
void static spi_endframe();
uint8_t static spi_readwrite(uint8_t tx_data);

// Configuration Options
#define RF_CHANNEL 20

void nrf24l01p_init() {
	// Configure IO
	NRF24L01P_MOSI_PORT.DIR |= NRF24L01P_MOSI_PIN;
	NRF24L01P_SCLK_PORT.DIR |= NRF24L01P_SCLK_PIN;
	NRF24L01P_CS_PORT.DIR |= NRF24L01P_CS_PIN;
	NRF24L01P_CE_PORT.DIR |= NRF24L01P_CE_PIN;
	
	// Configure SPI
	NRF24L01P_SPI.CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_CLK2X_bm;
	
	// Assert CS
	NRF24L01P_CS_PORT.OUT |= NRF24L01P_CS_PIN;
	
	nrf24l01p_wake();
	_delay_ms(1.5); // Wait 1.5ms per datasheet
	nrf24l01p_write_register(REG_RF_CH, RF_CHANNEL);
	nrf24l01p_write_register(REG_RF_SETUP, RF_SETUP_RF_DR);
	nrf24l01p_flush_rx_fifo();
	nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_wake() {
	nrf24l01p_write_register(REG_CONFIG, CONFIG_PWR_UP);
}

void nrf24l01p_flush_tx_fifo(void) {
	spi_startframe();
	spi_readwrite(SPICMD_FLUSH_TX);
	spi_endframe();
}

void nrf24l01p_flush_rx_fifo(void) {
	spi_startframe();
	spi_readwrite(SPICMD_FLUSH_RX);
	spi_endframe();
}

void nrf24l01p_write_register(uint8_t address, uint8_t new_value) {
	spi_startframe();
	spi_readwrite(SPICMD_W_REGISTER(address));
	spi_readwrite(new_value);
	spi_endframe();
}

void nrf24l01p_write_register_m(uint8_t address, uint8_t *new_value, size_t value_len) {
	spi_startframe();
	spi_readwrite(SPICMD_W_REGISTER(address));
	uint8_t i;
	for(i = 0; i < value_len && i < 32; i++) {
		spi_readwrite(new_value[i]);
	}
	spi_endframe();
}

void nrf24l01p_send_payload(uint8_t *data, size_t data_len) {
	spi_startframe();
	spi_readwrite(SPICMD_W_TX_PAYLOAD);
	uint8_t i;
	for(i = 0; i < data_len && i < 32; i++) {
		spi_readwrite(data[i]);
	}
	spi_endframe();
}

void nrf24l01p_data_test(void) {
	nrf24l01p_write_register(REG_EN_AA, 0x0);
	nrf24l01p_write_register(REG_SETUP_RETR, 0x0);
	uint8_t addr[] = {0xcc, 0xff, 0x00, 0xff, 0xcc};
	nrf24l01p_write_register_m(REG_TX_ADDR, addr, 5);
	while(1) {
		uint8_t test_data[32], i;
		
		uint8_t n = 0;
		for(i = 0; i < 32; i++) {
			test_data[i] = n++;
		}
		
		nrf24l01p_send_payload(test_data, 32);
		NRF24L01P_CE_PORT.OUT |= NRF24L01P_CE_PIN;
		_delay_us(20);
		NRF24L01P_CE_PORT.OUT &= ~NRF24L01P_CE_PIN;
	}
}

void static spi_startframe() {
	NRF24L01P_CS_PORT.OUT &= ~NRF24L01P_CS_PIN;
}

void static spi_endframe() {
	NRF24L01P_CS_PORT.OUT |= NRF24L01P_CS_PIN;
}

uint8_t static spi_readwrite(uint8_t tx_data) {
	NRF24L01P_SPI.DATA = tx_data;
	while(!(NRF24L01P_SPI.STATUS & SPI_IF_bm)); // Wait for transmission
	return NRF24L01P_SPI.DATA;
}