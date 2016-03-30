#include <iostream>
#include <errno.h>
#include <pigpio.h>
#include <stdint.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "build_bike.h"

using namespace std;

#define INT_PIN (4) //GPIO interrupt is detected on PIN 4
#define INT_PIN_ASLP (17)// Sleep interrupt is on PIN17  
// for some reason, adding 1 works
char MMA_result[6+1];
char MMA_check[1+1];
char MMA_int_src[1+1]; //interrupt source for MMA

int err; //error code
int ISR_OFF = 0;
bool MMA_start_flag = 0; //MMA start collection flag
bool MMA_stop_flag = 0;//MMA flag for stoping 

uint16_t accel[3];//actual accelerometer reading before normalization
float accel_g[3]; //accelerometer reading after normalization
//function prototype
float MMA_g_out(uint16_t accel_reading);
void MMA_ISR(int gpio, int level, uint32_t tick);
void ASLP_ISR(int gpio, int level, uint32_t tick);

char MMA_init_buf[] = {4, MMA8451_ADDR,  // set up Chip address
                       2, 7, 2, MMA8451_CTRL_REG1,    0x18, 3,    // put MMA in standby
                       2, 7, 2, MMA8451_CTRL_REG2,    0x04, 3,    // enable sleep mode (SLPE)
                       2, 7, 2, MMA8451_CTRL_REG1,    0xC0, 3,    // ASLP_rate = 1.56zm, AWKE_rate = 800Hz, no LNOISE, no_FREAD, standby
                       2, 7, 2, MMA8451_CTRL_REG4,    0x84, 3,    // Sleep/Wake + Motion interrupt enabled, non other interrupts enabled
                       2, 7, 2, MMA8451_CTRL_REG5,    0x80, 3,    // Route Pulse, Motion1 and Orientation to INT2 and Auto-Sleep to INT1.
                       2, 7, 2, MMA8451_CTRL_REG3,    0x08, 3,    // only Motion can wake device .
                       2, 7, 2, MMA8451_FF_MT_CFG,    0x78, 3,    //configure interrupt to detect freefall in XYZ direction, event flag latch disabled
                       2, 7, 2, MMA8451_FF_MT_THS,    0x1F, 3,    // detect 2 g acceleration in XYZ directions
                       2, 7, 2, MMA8451_FF_MT_COUNT,  0x18, 3,    // debounce ctr 30ms/1.25ms(@800Hz) = 24counts = 0x18
                       2, 7, 2, MMA8451_ASLP_COUNT,   0x1E, 3,    // enter sleep mode after 6 seconds inactivity
                       2, 7, 2, MMA8451_XYZ_DATA_CFG, 0x02, 3,    // Configure XYZ +/- 8g (full-scale  = 8g)
		                   2, 7, 2, MMA8451_CTRL_REG1,    0xC1, 3,	  //initialize MMA
                       0 // EOL
                       };
char MMA_read_int_buf[] = {4, MMA8451_ADDR,  // set up Chip address
                       2, 7, 1, MMA8451_FF_MT_SRC, 2, 6, 1, 3,  //read FF_MT_SRC register to clear event flag
                       0 // EOL
                       };
char MMA_read_buf[] = {4, MMA8451_ADDR,  // set up Chip address
                       2, 7, 1, MMA8451_OUT_X_MSB, 2, 6, 6, 3,  //read 6 bytes following MMA8451_OUT_X_MSB
                       0 // EOL
                       };

char MMA_check_buf[] = {4, MMA8451_ADDR,  // set up Chip address
                       2, 7, 1, MMA8451_WHO_AM_I ,2,6,1, 3, 
                       0 // EOL
                       };

int main(){

bbI2CClose(2);

gpioTerminate();

if (gpioInitialise() < 0) { //initialization of GPIO library

	printf("Pigpio library initialization failed\n");
	return -1;
}

if (bbI2COpen(2,3,100000) != 0){ //start Bitbanging on with SDA = pin2, SCL = pin3
	printf("Bit Banging initialization failed\n");
	return -1;
}

/*** Initialize MMA ***/

err = bbI2CZip(2,MMA_init_buf,sizeof(MMA_init_buf),NULL,0);
while (err != 0){
	err = bbI2CZip(2,MMA_init_buf,sizeof(MMA_init_buf),NULL,0);
	printf ("MMA init error = %d\n", err);
}
printf ("MMA init pass\n");

err = bbI2CZip(2,MMA_check_buf,sizeof(MMA_check_buf),MMA_check,sizeof(MMA_check) );
while (err != 1){
	err = bbI2CZip(2,MMA_check_buf,sizeof(MMA_check_buf),MMA_check,sizeof(MMA_check) );
	printf("MMA check ID error = %d\n", err);
}


if(MMA_check[0] == MMA8451_WHO_AM_I_VALUE){
	printf("MMA Sensor Connected, ID = %d\n", MMA_check[0]);
}else{
	printf("ERROR! MMA Sensor not found\n");
	while(bbI2CClose(2)!=0);
	gpioTerminate();
	return -1;
}

gpioSetISRFunc(INT_PIN, EITHER_EDGE,-1,MMA_ISR); //enable GPIO interrupt on Pin 4

while (!MMA_stop_flag){
	if (MMA_start_flag){
		ISR_OFF++;
		if(ISR_OFF ==1) {
			if((err = gpioSetISRFunc(INT_PIN, RISING_EDGE,0,NULL))!=0){
				printf("interrupt termination error = %d", err);
			}
       gpioSetISRFunc(INT_PIN_ASLP, EITHER_EDGE,-1,ASLP_ISR); //enable GPIO interrupt on Pin 4
		}
		//X_MSB,X_LSB,Y_MSB,Y_LSB,Z_MSB,Z_LSB
		err = bbI2CZip(2, MMA_read_buf,sizeof(MMA_read_buf),MMA_result,sizeof(MMA_result));
	
  		//Accel XYZ
		accel[0]= ((MMA_result[0]<<8) + MMA_result[1])&0xFFFC;
		accel[1]= ((MMA_result[2]<<8) + MMA_result[3])&0xFFFC;
		accel[2]= ((MMA_result[4]<<8) + MMA_result[5])&0xFFFC;
 		accel_g[0] = MMA_g_out(accel[0]);
 		accel_g[1] = MMA_g_out(accel[1]);
 		accel_g[2] = MMA_g_out(accel[2]);
  		if (err == 6){
			printf("X = %.4f  "   ,accel_g[0]);
			printf("Y = %.4f  "   ,accel_g[1]);
			printf("Z = %.4f  \n" ,accel_g[2]);
  		}
  		else{
  			printf("MMA read error. error = %d \n", err);
  		}
	}
}
  printf("collection finished!\n");
  bbI2CClose(2);

  gpioTerminate();
}

void MMA_ISR(int gpio, int level, uint32_t tick){
  printf("Interrupt Source = %d", MMA_int_src[0]);

  printf ("MMA_ISR - gpio: %d, level: %d\n", gpio, level);
  printf("hello world\n");
  MMA_start_flag = 1;

}

void ASLP_ISR(int gpio, int level, uint32_t tick){
  if(MMA_start_flag){//it has already started recording, now we can stop
      MMA_stop_flag = 1;
  } 
  else{
  // do nothing
  }
}

float MMA_g_out(uint16_t accel_reading){
	int accel_g_hi = 0;
	int accel_g_low = 0;
   bool negative = 0;
	/*
	** Determine sign and output
	*/
	if ((accel_reading & 0x8000)  == 0x8000){
		accel_reading &= 0xFFFC;
    negative = 1;
		accel_reading = ~accel_reading + 1;
	}
	/*
	** Determine integer value and output
	*/
	accel_g_hi = ((accel_reading>>8) & 0x70) >>4;
	/*
	** Determine mantissa value
	*/
	if ((accel_reading & 0x0800) == 0x0800) {
    accel_g_low += 5000;
  }
	if ((accel_reading & 0x0400) == 0x0400) {

      accel_g_low += 2500;
  }
	if ((accel_reading & 0x0200) == 0x0200) {
      accel_g_low += 1250;
  }
	if ((accel_reading & 0x0100) == 0x0100) {
      accel_g_low += 625;
  }
	if ((accel_reading & 0x0080) == 0x0080) {
      accel_g_low += 313;
  }
	if ((accel_reading & 0x0040) == 0x0040) {
      accel_g_low += 156;
  } 
	if ((accel_reading & 0x0020) == 0x0020) {
      accel_g_low += 78;
  }
	if ((accel_reading & 0x0010) == 0x0010) {
      accel_g_low += 39;
  }
	if ((accel_reading & 0x0008) == 0x0008) {
      accel_g_low += 20;
  }
	if ((accel_reading & 0x0004) == 0x0004) {
      accel_g_low += 10;
  }
	float accel_g_float = (float)accel_g_hi + ((float)accel_g_low)/10000;
 if (negative) accel_g_float = -accel_g_float;
	return accel_g_float;
}
