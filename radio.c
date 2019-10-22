/**
******************************************************************************
* @file    mylib/s4358870_radio.c
* @author  Julius Miyumo – s4358870
* @date    29032016
* @brief   radio FSM controller peripheral driver
*
******************************************************************************
*     EXTERNAL FUNCTIONS
******************************************************************************
* s4358870__radio_init() - Initialise	radio	(GPIO,	SPI,	etc)
* s4358870_radio_fsmprocessing() - Radio	FSM	processing	loop
* s4358870_radio_setchan(unsigned char chan) - Set	the	channel	of	the	radio
* s4358870_radio_settxaddress(unsigned char *addr) - Set	the	transmit	address	of	the	radio
* unsigned	char s4358870_radio_getchan() - Get	the	channel	of	the	radio
* s4358870_radio_gettxaddress(unsigned	char	*addr) - Get	the	transmit	address	of	the	radio
* s4358870_radio_sendpacket(char  chan, unsigned char *addr, unsigned char *txpacket) - Function	to	send	a	packet
* s4358870_radio_setfsmrx() - Set	Radio	FSM	into	RX	mode
* int s4358870_radio_getrxstatus() - Function	to	check	when	packet	is	received
* s4358870_radio_getpacket(unsigned char *rxpacket) - to receive	a	packet, when s4358870_radio_rxstatus is	1
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "nrf24l01plus.h"
#include "debug_printf.h"
#include "radio_fsm.h"
#include "s4358870_radio.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
STATE current_state; //Current state of FSM
uint8_t s4358870_tx_buffer[32];

/* Base addresses */
uint8_t b_addr_1[5] =  {0x78, 0x56, 0x34, 0x12, 0x00};
uint8_t b_addr_2[5] =  {0x79, 0x56, 0x34, 0x12, 0x00};
uint8_t b_addr_3[5] =  {0x7A, 0x56, 0x34, 0x12, 0x00};
uint8_t b_addr_4[5] =  {0x7B, 0x56, 0x34, 0x12, 0x00};

/* tx sent packet's MSB */
#define PACKET_TYPE 0xA1;
uint8_t source_addr[] = {0x43, 0x58, 0x87, 0x01};

extern void s4358870_radio_init() {
        BRD_init();
        BRD_LEDInit();		//Initialise Blue LED
        BRD_LEDOff();		//Turn off Blue LED
        radio_fsm_init(); //Initialise radio FSM
}

extern void s4358870_radio_fsmprocessing(void) {

        switch(current_state) {

                case S4358870_IDLE_STATE:	//Idle state for reading current channel
                        /* Get current channel , if radio FSM is in IDLE State */
                        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

                                if (s4358870_tx_packet_ready) {

                                        current_state = S4358870_TX_STATE;
                                } else {

                                        current_state = S4358870_RX_STATE;
                                }
                        } else {

                                /* if error occurs, set state back to IDLE state */
                                debug_printf("ERROR: Radio FSM not in Idle state\n\r");
                                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
                        }
                break;

                case S4358870_TX_STATE:	//TX state for writing packet to be sent.

                        /* Put radio FSM in TX state, if radio FSM is in IDLE state */
                        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

                                if (radio_fsm_setstate(RADIO_FSM_TX_STATE) == RADIO_FSM_ERROR) {
                                        debug_printf("ERROR: Cannot set Radio FSM TX state\n\r");
                                        HAL_Delay(100);
                                } else {

                                        radio_fsm_write(s4358870_tx_buffer);
                                        current_state = S4358870_IDLE_STATE;		//return to IDLE state
                                }
                        } else {

                                /* if error occurs, set state back to IDLE state */
                                debug_printf("ERROR: Radio FSM not in Idle state\n\r");
                                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
                        }
                break;

                case S4358870_RX_STATE:	//RX state for putting radio transceiver into receive mode.
                        /* Put radio FSM in RX state, if radio FSM is in IDLE or in waiting state */
                        if ((radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) || (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE)) {

                                if (radio_fsm_setstate(RADIO_FSM_RX_STATE) == RADIO_FSM_ERROR) {
                                        debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
                                        HAL_Delay(100);
                                } else {

                                        current_state = S4358870_WAITING_STATE;
                                }
                        } else {

                                /* if error occurs, set state back to IDLE state */
                                debug_printf("ERROR: Radio FSM not in Idle state\n\r");
                                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
                        }
                break;

                case S4358870_WAITING_STATE:	//Waiting state for reading received packet.

                        /* Check if radio FSM is in WAITING STATE */
                        if (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE) {

                                /* Check for received packet and display  */
                                if (radio_fsm_read(s4358870_rx_buffer) == RADIO_FSM_DONE) {

                                        s4358870_radio_rxstatus = 1; //packet recieved
                                        current_state = S4358870_RX_STATE;

                                } else {

                                        s4358870_radio_rxstatus = 0; // no packet recieved
                                }

                                if (s4358870_tx_packet_ready) {

                                        /* ready to transmit a packet*/
                                        radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
                                        current_state = S4358870_IDLE_STATE;
                                }
                        }
                break;
        }

}

extern void s4358870_radio_setchan(unsigned char chan) {

        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {
                radio_fsm_register_write(NRF24L01P_RF_CH, &chan);

        } else {

                /* if error occurs, set state back to IDLE state */
                debug_printf("ERROR: Cannot set channel Radio FSM not in Idle state\n\r");
                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
        }
}

extern unsigned char s4358870_radio_getchan() {
        uint8_t current_channel;
        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

                radio_fsm_register_read(NRF24L01P_RF_CH, &current_channel);
                return current_channel;
        } else {

                /* if error occurs, set state back to IDLE state */
                debug_printf("ERROR: Cannot get channel Radio FSM not in Idle state\n\r");
                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
        }
}

extern void s4358870_radio_settxaddress(unsigned char *addr) {
        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

                radio_fsm_buffer_write(NRF24L01P_TX_ADDR, addr, 5);
        } else {

                /* if error occurs, set state back to IDLE state */
                debug_printf("ERROR: Cannot set address Radio FSM not in Idle state\n\r");
                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
        }
}

extern void s4358870_radio_gettxaddress(unsigned	char *addr) {
        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

                radio_fsm_buffer_read(NRF24L01P_TX_ADDR, addr, 5);
        } else {

                /* if error occurs, set state back to IDLE state */
                debug_printf("ERROR: Cannot get address Radio FSM not in Idle state\n\r");
                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
        }
}

extern void s4358870_radio_sendpacket(char chan, unsigned char *addr, unsigned char *txpacket) {
        int i, n = 0;
        unsigned char desitnation_addr[5];

        /* Select base station's channel with its respective address */
        s4358870_radio_setchan(chan);
        s4358870_radio_settxaddress(addr);
        s4358870_radio_gettxaddress(desitnation_addr);

        /* fill packet */
        for (i = 0; i < 32; i++) {
                if (i == 0) {

                        s4358870_tx_buffer[i] = PACKET_TYPE;
                } else if (i > 0 && i <= 4) {

                        s4358870_tx_buffer[i] = desitnation_addr[(3 - n)];
                        n++;
                } else if(i > 4 && i <= 8) {

                        s4358870_tx_buffer[i] = source_addr[(i-5)];
                } else if (i > 8 && i < 16 && txpacket[(i-9)] != '\0') {

                        s4358870_tx_buffer[i] = txpacket[(i-9)];
                } else {
                        s4358870_tx_buffer[i] = '-';
                }
        }
        s4358870_radio_fsmprocessing(); // in IDLE state
        s4358870_radio_fsmprocessing(); // sending
        memset(txpacket,'\0', 16);

}

extern void s4358870_radio_setfsmrx(void) {

        if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {

                current_state = S4358870_RX_STATE;
                s4358870_radio_fsmprocessing(); // radio state set to RX

        } else {

                /* if error occurs, set state back to IDLE state */
                debug_printf("ERROR: Radio FSM cannot set to RX mode \n\r");
                radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
        }
}

extern int s4358870_radio_getrxstatus(void) {

        return s4358870_radio_rxstatus;
}

extern void s4358870_radio_getpacket(unsigned char *rxpacket) {
        int i;

        for (i = 0; i < 32; i++ ) {
                if (s4358870_rx_buffer[i] != '\0') {

                        rxpacket[i] = s4358870_rx_buffer[i];
                }
        }

        s4358870_radio_rxstatus = 0;
}