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
 * \defgroup cc2538-sht11 cc2538 sht11
 *
 * Driver for the cc2538 sht11 sensor
 * @{
 *
 * \file
 * Header file for the cc2538 sht11 driver
 *
 * \author
 *          Stevan Marinkovic <marinkovic.stevan@gmail.com>
 */
#ifndef SHT11_H_
#define SHT11_H_

/*
typedef union
{ unsigned int i;
  float f;
} value;
*/

//----------------------------------------------------------------------------------
// modul-var
//----------------------------------------------------------------------------------
#define TEMP	0
#define HUMI	1

#define	 DATA_BASE     GPIO_B_BASE
#define	 SCK_BASE      GPIO_B_BASE
#define DATA_PIN     3
#define SCK_PIN      5
#define	 DATA   	  0x08
#define	 SCK   	      0x20

#define	 PB_2   	  0x04
#define PB_4   	      0x10

#define noACK 0
#define ACK   1
                             //adr  command  r/w
#define STATUS_REG_W 0x06   //000   0011    0
#define STATUS_REG_R 0x07   //000   0011    1
#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
#define RESET        0x1e   //000   1111    0

// Function prototypes

char s_write_byte(unsigned char value);

char s_read_byte(unsigned char ack);

void s_transstart(void);

void s_connectionreset(void);

char s_softreset(void);

char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum);

char s_write_statusreg(unsigned char *p_value);

char s_measure(uint16_t *p_value, unsigned char *p_checksum, unsigned char mode);

void calc_sth11(float *p_humidity ,float *p_temperature);

float calc_dewpoint(float h,float t);

void gpio_test(void);

float sht11_TemperatureC(int rawdata);

float sht11_Humidity(int temprawdata,int humidityrawdata);



#endif /* SHT11_H_ */


/**
 * @}
 * @}
 */
