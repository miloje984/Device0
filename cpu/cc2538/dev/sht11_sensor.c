     /***********************************************************************************
Project:          SHT1x/7x demo program (V2.4)
Filename:         SHT1x_sample_code.c    

Prozessor:        80C51 family
Compiler:         Keil Version 6.23a

Autor:            MST
Copyrigth:        (c) Sensirion AG      
***********************************************************************************/
// Revisions:
// V2.4	 calc_sht11()       Coefficients for humidity and temperature conversion 
//                          changed (for V4 sensors)
//       calc_dewpoint()	New formula for dew point calculation 
 
#include "contiki.h"
#include "dev/i2c.h"
#include "sys/energest.h"
#include "dev/sys-ctrl.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "reg.h"
#include "sht11_sensor.h"
#include <math.h>    
#include <stdio.h> 
#include <string.h>
#include <stdint.h>

#if STARTUP_CONF_VERBOSE
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void gpio_test(void)
{
	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x01);
	GPIO_CLR_PIN(GPIO_C_BASE, 0x01);

	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x02);
	GPIO_SET_PIN(GPIO_C_BASE, 0x02);

	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x04);
	GPIO_SET_PIN(GPIO_C_BASE, 0x04);

	GPIO_SET_OUTPUT(GPIO_C_BASE, 0x08);
	GPIO_SET_PIN(GPIO_C_BASE, 0x08);
}

// writes a byte on the Sensibus and checks the acknowledge 
char s_write_byte(unsigned char value)
{
  unsigned char i,error=0;
  unsigned char val;

  val = value;

  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, SCK);
  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, DATA);
  // Set CLK pin as output
  GPIO_SET_OUTPUT(GPIO_B_BASE, SCK);
  // Set data pin as output
  GPIO_SET_OUTPUT(GPIO_B_BASE, DATA);
    
  for (i=0x80; i>0; i/=2)                       //shift bit for masking
  //for (i=0; i<8; i++, val <<= 1)
  { 
	//  if ((val & 0x80) == 0x80)
	//	  GPIO_SET_PIN(GPIO_B_BASE, DATA);
	//  else
	//	  GPIO_CLR_PIN(GPIO_B_BASE, DATA);
    if (i & value)                            //masking value with i , write to SENSI-BUS
    	GPIO_SET_PIN(GPIO_B_BASE, DATA);
    else
    	GPIO_CLR_PIN(GPIO_B_BASE, DATA);
                             
    clock_delay_usec(2);                      //observe setup time
    GPIO_SET_PIN(GPIO_B_BASE, SCK);         //clk for SENSI-BUS
    clock_delay_usec(2);                      //pulswith approx. 5 us
    GPIO_CLR_PIN(GPIO_B_BASE, SCK);
    clock_delay_usec(2);                      //observe hold time
  }
  
  GPIO_SET_PIN(GPIO_B_BASE, DATA);          //release DATA-line
  clock_delay_usec(2);                        //observe setup time
  GPIO_SET_PIN(GPIO_B_BASE, SCK);          //clk #9 for ack
  clock_delay_usec(2);
  
  GPIO_PERIPHERAL_CONTROL(DATA_BASE, DATA);
  // Set data pin as input
  GPIO_SET_INPUT(DATA_BASE, DATA);

  error = GPIO_READ_PIN(GPIO_B_BASE, DATA) >> DATA_PIN;    //check ack (DATA will be pulled down by SHT11)

  GPIO_CLR_PIN(GPIO_B_BASE, SCK);
  clock_delay_usec(2);

  return error;                                               //error=1 in case of no acknowledge
}

// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1" 
char s_read_byte(unsigned char ack)
{ 
  unsigned char i,val=0;

  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, SCK);
  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, DATA);
  // Set CLK pin as output
  GPIO_SET_OUTPUT(SCK_BASE, SCK);
  // Set data pin as output
  GPIO_SET_OUTPUT(DATA_BASE, DATA);

  GPIO_SET_PIN(GPIO_B_BASE, DATA);                          //release DATA-line
  
  GPIO_PERIPHERAL_CONTROL(DATA_BASE, DATA);
  // Set data pin as input
  GPIO_SET_INPUT(DATA_BASE, DATA);
  
  GPIO_CLR_PIN(GPIO_B_BASE, SCK);
  clock_delay_usec(2);

  for (i=0x80; i>0; i/=2)                                       //shift bit for masking
  { 
	  	GPIO_SET_PIN(GPIO_B_BASE, SCK);                           //clk for SENSI-BUS
	  	clock_delay_usec(2);
    
		if (GPIO_READ_PIN(GPIO_B_BASE, DATA) == DATA)            //read bit
		    val=(val | i);

		GPIO_CLR_PIN(GPIO_B_BASE, SCK);
		clock_delay_usec(2);
  }

  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, SCK);
  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, DATA);
  // Set data pin as output
  GPIO_SET_OUTPUT(DATA_BASE, DATA);

  //in case of "ack==1" pull down DATA-Line
  if (ack == 1)
	  GPIO_CLR_PIN(GPIO_B_BASE, DATA);
  else
	  GPIO_SET_PIN(GPIO_B_BASE, DATA);
  clock_delay_usec(2);                                          //observe setup time

  GPIO_SET_PIN(GPIO_B_BASE, SCK);                             //clk #9 for ack
  clock_delay_usec(5);                                          //pulswith approx. 5 us 
  GPIO_CLR_PIN(GPIO_B_BASE, SCK);
  clock_delay_usec(2);                                          //observe hold time						    
  GPIO_SET_PIN(GPIO_B_BASE, DATA);                          //release DATA-line
  clock_delay_usec(2);
  
  return val;
}

// generates a transmission start 
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
void s_transstart(void)
{
   GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, SCK);
   GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, DATA);
   // Set CLK pin as output
   GPIO_SET_OUTPUT(SCK_BASE, SCK);
   // Set data pin as output
   GPIO_SET_OUTPUT(DATA_BASE, DATA);
  
   GPIO_SET_PIN(GPIO_B_BASE, DATA);        //Initial state
   GPIO_CLR_PIN(GPIO_B_BASE, SCK);         //Initial state
   clock_delay_usec(2);
   
   GPIO_SET_PIN(GPIO_B_BASE, SCK);
   clock_delay_usec(2);
   GPIO_CLR_PIN(GPIO_B_BASE, DATA);
   clock_delay_usec(2);
   GPIO_CLR_PIN(GPIO_B_BASE, SCK);
   clock_delay_usec(2);
   GPIO_SET_PIN(GPIO_B_BASE, SCK);
   clock_delay_usec(2);
   GPIO_SET_PIN(GPIO_B_BASE, DATA);
   clock_delay_usec(2);
   GPIO_CLR_PIN(GPIO_B_BASE, SCK);
   clock_delay_usec(2);
}

// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
void s_connectionreset(void)
{  
  unsigned char i; 

  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, SCK);
  GPIO_SOFTWARE_CONTROL(GPIO_B_BASE, DATA);
  // Set CLK pin as output
  GPIO_SET_OUTPUT(SCK_BASE, SCK);
  // Set data pin as output
  GPIO_SET_OUTPUT(DATA_BASE, DATA);
  
  GPIO_SET_PIN(GPIO_B_BASE, DATA);        //Initial state
  GPIO_CLR_PIN(GPIO_B_BASE, SCK);         //Initial state
  clock_delay_usec(2);
   
  for(i=0; i<9; i++)                         //9 SCK cycles
  { 
	  GPIO_SET_PIN(GPIO_B_BASE, SCK);
	  clock_delay_usec(2);
	  GPIO_CLR_PIN(GPIO_B_BASE, SCK);
	  clock_delay_usec(2);
  }
  
  s_transstart();                            //transmission start
}

// resets the sensor by a softreset 
char s_softreset(void)
{ 
  unsigned char error=0; 
   
  s_connectionreset();              //reset communication
  error += s_write_byte(RESET);       //send RESET-command to sensor
  
  return error;                     //error=1 in case of no response form the sensor
}

// reads the status register with checksum (8-bit)
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
{ 
  unsigned char error=0;
  
  s_transstart();                   //transmission start
  error=s_write_byte(STATUS_REG_R); //send command to sensor
  *p_value=s_read_byte(ACK);        //read status register (8-bit)
  *p_checksum=s_read_byte(noACK);   //read checksum (8-bit)  
  
  return error;                     //error=1 in case of no response form the sensor
}

// writes the status register with checksum (8-bit)
char s_write_statusreg(unsigned char *p_value)
{ 
  unsigned char error=0;
  
  s_transstart();                   //transmission start
  error+=s_write_byte(STATUS_REG_W);//send command to sensor
  error+=s_write_byte(*p_value);    //send value of status register
  
  return error;                     //error>=1 in case of no response form the sensor
}
 							   
// makes a measurement (humidity/temperature) with checksum
char s_measure(uint16_t *p_value, unsigned char *p_checksum, unsigned char mode)
{ 
  unsigned char error=0;
  unsigned int i;
  unsigned char data_low, data_high;

  s_transstart();                   //transmission start
  switch(mode)                      //send command to sensor
  {                     
    case TEMP: 
      error+=s_write_byte(MEASURE_TEMP); 
    break;
    
    case HUMI: 
      error+=s_write_byte(MEASURE_HUMI); 
    break;
    
    default: 
    break;	 
  }
  
  for (i=0; i<30000; i++)     //wait until sensor has finished the measurement
  {
	  clock_delay_usec(100);
      if (GPIO_READ_PIN(GPIO_B_BASE, DATA) == 0)
    	  break;
  }
  
  if (GPIO_READ_PIN(GPIO_B_BASE, DATA) == DATA)         // or timeout (~2 sec.) is reached
    error+=1; 
                   
  data_high = s_read_byte(ACK);    //read the first byte (MSB)
  data_low = s_read_byte(ACK);    //read the second byte (LSB)
  *p_checksum = s_read_byte(noACK);  //read checksum
  
  if (mode == TEMP)
  {
	  *p_value = (uint16_t)data_high;
	  *p_value <<= 8;
	  *p_value |= (uint16_t)data_low;
  }

  if (mode == HUMI)
  {
	  *p_value = (uint16_t)data_high;
	  *p_value <<= 8;
	  *p_value |= (uint16_t)data_low;
  }

  return error;
}

// calculates temperature [\B0C] and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [\B0C]
void calc_sth11(float *p_humidity ,float *p_temperature)
{ 
  const float C1=-2.0468;           // for 12 Bit RH
  const float C2=+0.0367;           // for 12 Bit RH
  const float C3=-0.0000015955;     // for 12 Bit RH
  const float T1=+0.01;             // for 12 Bit RH
  const float T2=+0.00008;          // for 12 Bit RH	

  float rh=*p_humidity;             // rh:      Humidity [Ticks] 12 Bit 
  float t=*p_temperature;           // t:       Temperature [Ticks] 14 Bit
  float rh_lin;                     // rh_lin:  Humidity linear
  float rh_true;                    // rh_true: Temperature compensated humidity
  float t_C;                        // t_C   :  Temperature [\B0C]

  t_C = t*0.01 - 40.1;                    //calc. temperature [\B0C] from 14 bit temp. ticks @ 5V

  rh_lin = C3*rh*rh + C2*rh + C1;         //calc. humidity from ticks to [%RH]
  rh_true = (t_C-25)*(T1+T2*rh)+rh_lin;   //calc. temperature compensated humidity [%RH]
  
  if(rh_true>100)                   //cut if the value is outside of
    rh_true = 100;
    
  if(rh_true<0.1)                   //the physical possible range
    rh_true = 0.1;

  *p_temperature = t_C;             //return temperature [\B0C]
  *p_humidity = rh_true;            //return humidity[%RH]
}

// calculates dew point
// input:   humidity [%RH], temperature [\B0C]
// output:  dew point [\B0C]
float calc_dewpoint(float h,float t)
{ 
  float k, dew_point ;
  
  k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
  dew_point = 243.12*k/(17.62-k);
  
  return dew_point;
}
