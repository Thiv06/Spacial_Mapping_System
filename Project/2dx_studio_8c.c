/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Thivagar Yogaratnam , yogaratt, 400213615, L04

						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file

*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

void PortJ_Init(void){ //ON-BOARD PSUH BUTTON
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){   // TOF INITIALIZATION
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}
void PortH_Init(void){   //STEPPER MOTOR
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; //allow time for clock to stabilize
	GPIO_PORTH_DIR_R = 0b00001111; // Make PM0:PM3 inputs, reading if the button is pressed or not
	GPIO_PORTH_DEN_R = 0b00001111; // Enable PM0:PM3
return;
}
void PortM_Init(void){      //USED FOR OFF BUTTON(PM0)
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x00	;        								// making PM0 an input  
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								// disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								// enable digital I/O on PN0
																									
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								// disable analog functionality on PN0		
	return;
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}

void spin_Clockwise()						//function to spin clockwise 45 degrees
{
		for(int i=0; i<64; i++){						//one full rotation through full stepping takes 2048 steps
			GPIO_PORTH_DATA_R = 0b00001001;			//therefore 1/8 of rotation is 45 deg is 256 steps
			SysTick_Wait10ms(2);								//since we have 4 steps in the loop, i= 256/4 = 64
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(2);
	}
}

void spin_CounterClockwise()						//function to spin clockwise 45 degrees
{
		for(int i=0; i<64; i++){						//one full rotation through full stepping takes 2048 steps
			GPIO_PORTH_DATA_R = 0b00001100;		//therefore 1/8 of rotation is 45 deg is 256 steps
			SysTick_Wait10ms(2);							//since we have 4 steps in the loop, i= 256/4 = 64
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(2);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(2);
		}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
int direction = 0;  // USING TO PRVENT ENTANGLING


int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t id;
	uint8_t type;
	uint16_t both;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortJ_Init();
	PortM_Init();
	




	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
  // OPTIONAL 
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */


  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	
	
	// Get the Distance Measures 8 times
	int counter = 0;
	while(counter < 3){ // 3 SCANS
		if(GPIO_PORTJ_DATA_R == 0b00000000){
			counter++;
			for(int i = 0; i < 8; i++) {       //EXTRACT 8 MEASUREMENTS
				
				if((GPIO_PORTM_DATA_R&0b00000010)==0){
					counter = 10; // stop taking measurements/rotating
					break;
				}
				else{
				}
				
				//wait until the ToF sensor's data is ready
				while (dataReady == 0){
					status = VL53L1X_CheckForDataReady(dev, &dataReady); // check if data has
					VL53L1_WaitMs(dev, 5);
				}
				
				dataReady = 0;
				status = VL53L1X_GetDistance(dev, &Distance);					//VALUE OF R

				if(direction%2==0){  // avoid entanglement
					spin_Clockwise();	
					FlashLED1(1);
				}else {
					spin_CounterClockwise();
					FlashLED1(1);					
				}
						
				status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
				
				// print the resulted readings to UART (SEEN IN PYTHON SHELL)
				sprintf(printf_buffer,"%u\n",Distance);
				UART_printf(printf_buffer);
				SysTick_Wait10ms(50);
			}
		}else{
			SysTick_Wait10ms(50);
		}
		direction++; //  allow to rotate CW/CCW
	}
  
	VL53L1X_StopRanging(dev); /// stop sensor
  while(1) {}

}
