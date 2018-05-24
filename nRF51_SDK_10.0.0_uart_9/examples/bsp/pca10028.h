/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef PCA10028_H
#define PCA10028_H



//PENG----

#define gpio_led          17
#define gpio_key       		18

//PENG====*/



#define RX_PIN_NUMBER  29	//
#define TX_PIN_NUMBER  28	//
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           false

// serialization APPLICATION board
#define SER_CONN_CHIP_RESET_PIN     30    // Pin used to reset connectivity chip

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM

#endif // PCA10028_H





