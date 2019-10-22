/**
  ******************************************************************************
  * @file    stage4/main.c
  * @author  Julius Miyumo
  * @date    25-March-2016
  * @brief   Prac 4 C main file - Radio Communications FSM controller
  *
  *	REFERENCES: ex10_spi, ex11_console, ex13_radio, STM32F4xx Datasheet, nRF24L01+ datasheet.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4358870_radio.h"
#include "radio_fsm.h"
#include "nrf24l01plus.h"

/* Private variables -----------------------------------------------*/
unsigned char tx_packet[21];

/* Private function prototypes -----------------------------------------------*/
int fill_tx_packet(void); //fill_packet to be transmitted
void display_packet(unsigned char *packet); // display recieved packet in kermusb

void main() {
    unsigned char channel;
    unsigned char address[5];
    unsigned char rxpacket[32];
    int i;
    uint8_t base_channel = BASE_CHN_3;
    uint8_t *base_addr = b_addr_3;

    uint32_t previous_time, time_passed;

    s4358870_radio_init();
    radio_fsm_setstate(RADIO_FSM_IDLE_STATE);

    s4358870_radio_setchan(base_channel);
    s4358870_radio_settxaddress(base_addr);
    channel = s4358870_radio_getchan();
    s4358870_radio_gettxaddress(address);
    HAL_Delay(3000);
    debug_printf("Channel: %d\n\r", channel);
    debug_printf("addresses:");
    for (i = 0; i < 5; i++) {
      debug_printf("%x", address[(4-i)]);
    }
    debug_printf("\n\r");
    s4358870_radio_setfsmrx(); //set RX mode
    s4358870_radio_fsmprocessing(); // wait for packet

  while (1) {

      if (s4358870_radio_getrxstatus() == 1) { // RECIEVED packet

          s4358870_radio_getpacket(rxpacket);
          display_packet(rxpacket);
          memset(s4358870_rx_buffer,'-', 16);
      } else {

          s4358870_tx_packet_ready = fill_tx_packet(); //  console -> TX
          if (s4358870_tx_packet_ready) { // ready to transmit

              s4358870_radio_fsmprocessing(); // next state is TX state
              s4358870_radio_sendpacket(base_channel, base_addr, tx_packet);
              s4358870_tx_packet_ready = 0;
              s4358870_radio_setfsmrx(); //set back to RX mode
              s4358870_radio_fsmprocessing(); // Waiting for packet
              BRD_LEDToggle();
          } else {

              s4358870_radio_fsmprocessing(); // stay in wait state
          }
      }

    }
}

void display_packet(unsigned char *data) {

  int i;
  debug_printf("RECV:");

  for (i = 0; i < 32; i++ ) {
      if (i > 4 && i < 9) {

        debug_printf("%x", data[(13 - i)]); // source addr
      } else if ( i == 9 ) {

          debug_printf(">");
      } else if (i > 9 && i < 16) {

         debug_printf("%c-", data[(i-1)]); // payload
      }
  }

  debug_printf("\n\r");
}

int fill_tx_packet(void) {
  char *txChar;

  while(i < 21) {

    char txChar = debug_getc();

    if (txChar == '1') {
      key = TASK_1;
      continue;
    } else if (txChar == '2') {
      key = TASK_2;
      continue;
    }

    if ((int)txChar != 13 && txChar != '\0') {

      i++;
      /* fill packet continously */
      tx_packet[i] = txChar;
    } else if (i == 0 || txChar == '\0') {

      /* No packet ready to be sent*/
      return 0;
    } else if ((int)txChar == 13 || i == 21) {

      i = -1;
      /* packet ready to be sent*/
      return 1;
    }
  }

  /* enter key not yet pushed */
  return 0;
}