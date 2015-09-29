/*
 * nrf24l01p.h
 *
 * Created: 9/25/2015 1:54:36 PM
 *  Author: Nigil Lee
 */ 

#include <stdint.h>
#include <string.h>

#ifndef NRF24L01P_H_
#define NRF24L01P_H_

void nrf24l01p_init();
void nrf24l01p_wake();
void nrf24l01p_flush_tx_fifo(void);
void nrf24l01p_flush_rx_fifo(void);
void nrf24l01p_write_register(uint8_t address, uint8_t new_value);
void nrf24l01p_write_register_m(uint8_t address, uint8_t *new_value, size_t value_len);
void nrf24l01p_send_payload(uint8_t *data, size_t data_len);
void nrf24l01p_data_test(void);

#endif /* NRF24L01P_H_ */