/*
 * rf_transceiver.c
 *
 * Created: 9/25/2015 1:49:59 PM
 *  Author: Nigil Lee
 */

#include <avr/io.h>
#include <util/delay.h>
#include "drivers/nrf24l01p.h"

int main(void)
{
	nrf24l01p_init();
	PORTA.DIR |= PIN0_bm;
	PORTA.OUT |= PIN0_bm;
	
    while(1)
    {
        nrf24l01p_data_test();
    }
}