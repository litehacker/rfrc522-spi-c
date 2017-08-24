/** RFID MAHMOTE COMMUNICATION
  * Developed by Giorgi Gvimradze for mavialp
*/
#include "rfid-spi.h"
#include "contiki-net.h"

uid_struct uid; 

PROCESS(rfid_proccess, "Hello world process");
AUTOSTART_PROCESSES(&rfid_proccess);

PROCESS_THREAD(rfid_proccess, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  static struct etimer et;
  
  etimer_set(&et, CLOCK_SECOND * 0.5);

  GPIO_SET_OUTPUT(GPIO_A_BASE, GPIO_PIN_6);
  GPIO_SET_PIN(GPIO_A_BASE, GPIO_PIN_6);    // turn the led on to determine the meter is synched up.

    while(1){
      PROCESS_YIELD();
      if(ev==PROCESS_EVENT_TIMER)
      {
        if(etimer_expired(&et))
        {
          etimer_reset(&et);
        }
      }
      if (pcd_initialization())
        break;
    }

    while(1)
    {
      PROCESS_YIELD();
      write_mfrc522(ErrorReg,0x00);
      if(ev==PROCESS_EVENT_TIMER)
      {
        if(etimer_expired(&et))
        {
          etimer_reset(&et);
        }
      }
      printf("start main\n");


      // Look for new cards
      if (!picc_is_new_card_present() ) {
        continue;
      }
      // Select one of the cards
      if (!picc_read_card_serial()) {
        continue;
      }

      // Dump debug info about the card; picc_halt_a() is automatically called
      printf("\nCARD UID SHIFTED: %x \n",    picc_dump_to_serial(&uid));
      net_debug_lladdr_print(&uip_lladdr);
      // If you need to read 1KB data from a card use picc_dump_contents(&uid); function.
      // The function needs to be updated.
    }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/