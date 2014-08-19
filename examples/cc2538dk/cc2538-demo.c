/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538
 * @{
 *
 * \defgroup cc2538-examples cc2538dk Example Projects
 * @{
 *
 * \defgroup cc2538-demo cc2538dk Demo Project
 *
 *   Example project demonstrating the cc2538dk functionality
 *
 *   This assumes that you are using a SmartRF06EB with a cc2538 EM
 *
 * - Boot sequence: LEDs flashing, LED2 followed by LED3 then LED4
 * - etimer/clock : Every LOOP_INTERVAL clock ticks the LED defined as
 *                  LEDS_PERIODIC will turn on
 * - rtimer       : Exactly LEDS_OFF_HYSTERISIS rtimer ticks later,
 *                  LEDS_PERIODIC will turn back off
 * - Buttons      :
 *                - BTN_DOWN turns on LEDS_REBOOT and causes a watchdog reboot
 *                - BTN_UP to soft reset (SYS_CTRL_PWRDBG::FORCE_WARM_RESET)
 *                - BTN_LEFT and BTN_RIGHT flash the LED defined as LEDS_BUTTON
 * - UART         : Every LOOP_INTERVAL the EM will print something over the
 *                  UART. Receiving an entire line of text over UART (ending
 *                  in \\r) will cause LEDS_SERIAL_IN to toggle
 * - Radio comms  : BTN_SELECT sends a rime broadcast. Reception of a rime
 *                  packet will toggle LEDs defined as LEDS_RF_RX
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc2538dk platform
 */
#include "contiki.h"
#include "cpu.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "dev/uart.h"
#include "dev/button-sensor.h"
#include "dev/watchdog.h"
#include "dev/serial-line.h"
#include "dev/sys-ctrl.h"
/* sht11 include  */
#include "dev/sht11_sensor.h"
#include "net/rime/broadcast.h"

/* adc include */
#include "dev/adc.c"

#include <stdio.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
#define LOOP_INTERVAL       CLOCK_SECOND
#define LEDS_OFF_HYSTERISIS (RTIMER_SECOND >> 1)
#define LEDS_PERIODIC       LEDS_YELLOW
#define LEDS_BUTTON         LEDS_RED
#define LEDS_SERIAL_IN      LEDS_ORANGE
#define LEDS_REBOOT         LEDS_ALL
#define LEDS_RF_RX          (LEDS_YELLOW | LEDS_ORANGE)
#define BROADCAST_CHANNEL   129
/*---------------------------------------------------------------------------*/
static struct etimer et;
static struct rtimer rt;
static uint16_t counter;
/* SHT11 */
 unsigned char err = 0;
	uint16_t temperature = 0;
	uint16_t temperature_r = 0;
	uint16_t humidity = 0;
	uint16_t humidity_r = 0;
	float* temperature_f;
	float* humidity_f;
	float tc;
	float tc_r;
	float hc;
	float hc_r;
	unsigned char* checksum;
	unsigned char status;
	unsigned char temp_data = 0xff;
	uint16_t swt=0;
	uint16_t rwt=0;
	 uint16_t ui1Dummy;
/*---------------------------------------------------------------------------*/
PROCESS(cc2538_demo_process, "cc2538 demo process");
AUTOSTART_PROCESSES(&cc2538_demo_process);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  leds_toggle(LEDS_RF_RX);
  //printf("Received %u bytes: '0x%04x'\n", packetbuf_datalen(),
  //       *(uint16_t *)packetbuf_dataptr());
  if (rwt == 0)
  {
  temperature_r=*(uint16_t *)packetbuf_dataptr();
  tc_r=sht11_TemperatureC(temperature_r);
  printf("temperature_received:%u.%u\n",(int)tc_r,((int)(tc_r*10))%10);
  rwt=1;
  }
  else
  {
  humidity_r=*(uint16_t *)packetbuf_dataptr();
  hc_r=sht11_Humidity(temperature_r,humidity_r);
  printf("humidity_received:%u.%u\n",(int)hc_r,((int)(hc_r*10))%10);
  rwt=0;
  }
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks bc_rx = { broadcast_recv };
static struct broadcast_conn bc;
/*---------------------------------------------------------------------------*/
void
rt_callback(struct rtimer *t, void *ptr)
{
  leds_off(LEDS_PERIODIC);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2538_demo_process, ev, data)
{

  PROCESS_EXITHANDLER(broadcast_close(&bc))

  PROCESS_BEGIN();

  counter = 0;
  broadcast_open(&bc, BROADCAST_CHANNEL, &bc_rx);
  printf("temp:%u.%u\nhumidity:%u.%u\n",(int)tc,((int)(tc*10))%10 , (int)hc,((int)(hc*10))%10);
  while(1) {
    etimer_set(&et, CLOCK_SECOND*10);

    //
    // Configure ADC, Internal reference, 512 decimation rate (12bit)
    //
    SOCADCSingleConfigure(SOCADC_12_BIT, SOCADC_REF_INTERNAL);
    //
    // Trigger single conversion on AIN6 (connected to LV_ALS_OUT).
    //
    SOCADCSingleStart(SOCADC_VDD);
    //
    // Wait until conversion is completed
    //
    while(!SOCADCEndOfCOnversionGet())
    {
    }
    //
    // Get data and shift down based on decimation rate
    //
    ui1Dummy = SOCADCDataGet() >> SOCADC_12_BIT_RSHIFT;
    printf("konverzija(ADC) = 0x%08x\n",ui1Dummy);
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      leds_on(LEDS_PERIODIC);
      printf("Counter = 0x%08x\n", counter);
      err = s_measure(&temperature, checksum, TEMP);

             	  			if (err == 0)
             	  			{
             	  				//printf("Temperature (ADC value) = 0x%4x\n", temperature);

             	  				err = s_measure(&humidity, checksum, HUMI);

             	  				if (err == 0)
             	  				{
             	  					//printf("Humidity (ADC value) = 0x%4x\n", humidity);

             	  					//tc=sht11_TemperatureC(temperature);
             	  					//hc=sht11_Humidity(temperature,humidity);
             	  					tc=0;
             	  					hc=0;
             	  					printf("temp:%u.%u\nhumidity:%u.%u\n",(int)tc,((int)(tc*10))%10 , (int)hc,((int)(hc*10))%10);
             	  				}
             	  				else
             	  					printf("SHT11 error - could not read humidity!\n");
             	  			}
             	  			else
             	  				printf("SHT11 error - could not read temperature!\n");

      etimer_set(&et, CLOCK_SECOND);
      rtimer_set(&rt, RTIMER_NOW() + LEDS_OFF_HYSTERISIS, 1,
                 rt_callback, NULL);
    } else if(ev == sensors_event) {
      if(data == &button_select_sensor) {
    	  if (swt==0)
    	  {
    		packetbuf_copyfrom(&temperature, sizeof(temperature));
    	  	broadcast_send(&bc);
        	swt=1;
    	  }
          else
          {
        	packetbuf_copyfrom(&humidity, sizeof(humidity));
    	  	broadcast_send(&bc);
        	swt=0;
          }

      } else if(data == &button_left_sensor || data == &button_right_sensor) {
        leds_toggle(LEDS_BUTTON);
      } else if(data == &button_down_sensor) {
        cpu_cpsid();
        leds_on(LEDS_REBOOT);
        watchdog_reboot();
      } else if(data == &button_up_sensor) {
        sys_ctrl_reset();
      }
    } else if(ev == serial_line_event_message) {
      leds_toggle(LEDS_SERIAL_IN);
    }
    counter++;
    /* put measaruement sht11 here*/

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
