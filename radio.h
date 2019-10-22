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

#ifndef S4358870_RADIO_H

/* Private define ------------------------------------------------------------*/
#define S4358870_RADIO_H

/* Base RF channels */
#define BASE_CHN_1 43
#define BASE_CHN_2 46
#define BASE_CHN_3 48
#define BASE_CHN_4 50

/* Private macro -------------------------------------------------------------*/
typedef enum {
  S4358870_IDLE_STATE = 0, //IDLE	state	(used	for	reading/writing	registers)
  S4358870_RX_STATE = 1, //Put	radio	FSM	into	receiving	mode
  S4358870_TX_STATE = 2, //Put	radio	FSM	into	transmitting	mode
  S4358870_WAITING_STATE = 3 //Wait	for	radio	FSM	to	receive	packet
} STATE;

typedef enum {
  BASE_1 = 1,
  BASE_2 = 2,
  BASE_3 = 3
} BASE;

typedef enum {
  TASK_1 = 0, // send name payload every 5 seconds passed
  TASK_2 = 1
} KEY;


/* Private variables ---------------------------------------------------------*/
int s4358870_radio_fsmcurrentstate; //Hold	current	state	of FSM
int s4358870_radio_rxstatus; //Status of radio RX (0 = no packet received, 1 = packet received)
unsigned char s4358870_rx_buffer[32]; //Radio	RX	buffer,	used	to	hold	received	packet
int s4358870_tx_packet_ready;

/* external variables */
extern uint8_t b_addr_1[5];
extern uint8_t b_addr_2[5];
extern uint8_t b_addr_3[5];
extern uint8_t b_addr_4[5];
extern STATE current_state;

/* External function prototypes -----------------------------------------------*/
extern void s4358870_radio_init(void);
extern void s4358870_radio_fsmprocessing(void);
extern void s4358870_radio_setchan(unsigned char chan);
extern void s4358870_radio_settxaddress(unsigned char *addr);
extern unsigned	char s4358870_radio_getchan(void);
extern void s4358870_radio_gettxaddress(unsigned	char *addr);
extern void s4358870_radio_sendpacket(char chan, unsigned char *addr, unsigned char *txpacket);
extern void s4358870_radio_setfsmrx(void);
extern int s4358870_radio_getrxstatus(void);
extern void s4358870_radio_getpacket(unsigned char *rxpacket);
 #endif