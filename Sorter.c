/*
 * GccApplication1.c
 *
 * Created: 2024-03-18 5:21:56 PM
 * Author : mech458
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>
#include "lcd.h"

// Global Variable
volatile unsigned int STATE;
volatile unsigned char ADC_resultH;
volatile unsigned char ADC_resultL;
volatile unsigned int ADC_result;
volatile unsigned int ADC_result_Flag=0;
volatile unsigned int ADC_min=0xFFF; // 16_bit to save 10-bit ADC reflectness
volatile unsigned int pauseflag = 0;
volatile unsigned int itemready = 0;
//#define STEP1 0b00110110;
//#define STEP2 0b00101110;
//#define STEP3 0b00101101;
//#define STEP4 0b00110101;
volatile unsigned int stepper[4] = {0b00110110,0b00101110,0b00101101,0b00110101};
//#define CLOCKWISE 1;
//#define COUNTERCLOCKWISE -1;
volatile unsigned int stepper_State;			// s1, s2, s3 ou s4
volatile unsigned int CurrentPosition;		// AL0, Blk1, Whit2, STL3, Unknown4
volatile unsigned int DesiredPosition;
volatile unsigned int Hall_Flag;
volatile unsigned int currentStep;

// For optical sensor OR
unsigned int Count_OptOR = 0;
volatile unsigned int OR_Flag = 0;

// For optical sensor EX
volatile unsigned int Count_OptEX = 0;
volatile unsigned int Exit_Flag = 0;

//Item List
typedef struct cylinderMATERIAL{
	int category;		// AL = 0; STL = 2; WPL = 3; BPL = 1; UNKnown = 4;
};
struct cylinderMATERIAL cylin[52];

volatile unsigned int cat;

// Cylinders count
volatile unsigned int AL_SortedCount = 0;
volatile unsigned int STL_SortedCount = 0;
volatile unsigned int WPL_SortedCount = 0;
volatile unsigned int BPL_SortedCount = 0;
volatile unsigned int Onbelt;
volatile unsigned int TotalSorted;


/* DC motor */
volatile unsigned char dutyCycle = 0x80; 	// set PWM = 50%
volatile unsigned char DC_cw = 0b00001011;	//CW (reverse)
volatile unsigned char DC_ccw = 0b00000111; //CCW (forward)
volatile unsigned char DC_Hstop = 0b00001111;  	// stop DC motor high
unsigned char system_state=0;


void configIO();
void configInterrupts();
void configPWM();
void configADC();
void DCMotorCtrl();
void ADC_calibrate();
void mTimer();
void stepper_Home();
void stepperRotate();
int stepperSorting(int CurrentPosition, int DesiredPosition);
void rTimer();



int main(int argc, char *argv[]){
	
	CLKPR = 0x80;
	CLKPR = 0x01;		//  sets system clock to 8MHz
	TCCR1B |= _BV(CS11);
	
	STATE = 0;

	cli();		// Disables all interrupts
	
	//DDRD = 0b11110000;	// Going to set up INT2 & INT3 on PORTD
	DDRC = 0xFF;		// just use as a display
	


	// Set up the Interrupt 0,3 options
	//External Interrupt Control Register A - EICRA (pg 110 and under the EXT_INT tab to the right
	// Set Interrupt sense control to catch a rising edge
	EICRA |= _BV(ISC21) | _BV(ISC20);
	EICRA |= _BV(ISC31) | _BV(ISC30);

	//	EICRA &= ~_BV(ISC21) & ~_BV(ISC20); /* These lines would undo the above two lines */
	//	EICRA &= ~_BV(ISC31) & ~_BV(ISC30); /* Nice little trick */


	// See page 112 - EIFR External Interrupt Flags...notice how they reset on their own in 'C'...not in assembly
	EIMSK |= 0x0C;

	configIO();
	configInterrupts();
	configPWM(dutyCycle);
	configADC();

	// Enable all interrupts
	sei();	// Note this sets the Global Enable for all interrupts

	InitLCD(LS_BLINK|LS_ULINE); //Initialize LCD module
	LCDClear(); //Clear the screen
	//LCDWriteString("hello world");
	//mTimer(5000);
	//LCDClear();
	
	stepper_Home();
	stepperRotate(8,1);
	DCMotorCtrl(0);
	//ADC_calibrate();
	goto POLLING_STAGE;
	
	// POLLING STAGE
	POLLING_STAGE:
	DCMotorCtrl(0);
	switch(STATE){
		case (0) :
		goto POLLING_STAGE;
		break;	//not needed but syntax is correct
		case (1) :
		goto PAUSE_BUTTON;
		break;
		case (2) :
		goto CALIBRATION_STAGE;
		break;
		case (3) :
		goto BUCKET_STAGE;
		break;
		case (5) :
		goto RAMPDOWN;
		default :
		goto POLLING_STAGE;
	}//switch STATE
	

	PAUSE_BUTTON:
	// Do whatever is necessary HERE
	//PORTC = 0x01; // Just output pretty lights know you made it here
	//Reset the state variable
	DCMotorCtrl(1);
	LCDClear();
	LCDWriteStringXY(0,0,"ST");
	LCDWriteIntXY(0,1,STL_SortedCount,2);
	LCDWriteStringXY(3,0,"BL");
	LCDWriteIntXY(3,1,BPL_SortedCount,2);
	LCDWriteStringXY(6,0,"AL");
	LCDWriteIntXY(6,1,AL_SortedCount,2);
	LCDWriteStringXY(9,0,"WH");
	LCDWriteIntXY(9,1,WPL_SortedCount,2);
	LCDWriteStringXY(12,0,"PS");
	LCDWriteIntXY(12,1,(Count_OptOR-Count_OptEX),2);
	while(pauseflag == 1);
	LCDClear();
	DCMotorCtrl(0);
	STATE = 0;
	goto POLLING_STAGE;

	CALIBRATION_STAGE:
	// Do whatever is necessary HERE
	//PORTC = 0x04; // Just output pretty lights know you made it here
	//Reset the state variable
	
	for(int j=0;j<4;j++){ //4 total trials, one for each category
		int cal_min = 0xFFFF;
		for(int i=0;i<10;i++){// run part through 10 times
			
			while(itemready == 0);

			LCDWriteIntXY(0,0,ADC_min,3);
			//mTimer(200);
			//LCDClear();
			if(ADC_min < cal_min){
				cal_min = ADC_min;
				LCDWriteIntXY(12,1,cal_min,3);
			}
			ADC_min = 0xFFFF;
			itemready = 0;
		}
		LCDClear();
	}
	
	STATE = 0;
	goto POLLING_STAGE;
	
	BUCKET_STAGE:
	
	DCMotorCtrl(1);			// stop the belt
	DesiredPosition = cylin[Count_OptEX-1].category;
	switch(DesiredPosition){
		case(0):
		STL_SortedCount++;
		break;
		case(1):
		BPL_SortedCount++;
		break;
		case(2):
		AL_SortedCount++;
		break;
		case(3):
		WPL_SortedCount++;
		break;
		default:
		break;
	}
	//LCDWriteStringXY(0,1,"here");
	CurrentPosition = stepperSorting(CurrentPosition, DesiredPosition);
	mTimer(20);
	STATE = 0;
	goto POLLING_STAGE;
	
	RAMPDOWN:
	// The closing STATE ... how would you get here?
	//PORTC = 0xF0;	// Indicates this state is active
	// Stop everything here...'MAKE SAFE'
	DCMotorCtrl(1);
	cli();
	LCDClear();
	LCDWriteStringXY(0,0,"ST");
	LCDWriteIntXY(0,1,STL_SortedCount,2);
	LCDWriteStringXY(3,0,"BL");
	LCDWriteIntXY(3,1,BPL_SortedCount,2);
	LCDWriteStringXY(6,0,"AL");
	LCDWriteIntXY(6,1,AL_SortedCount,2);
	LCDWriteStringXY(9,0,"WH");
	LCDWriteIntXY(9,1,WPL_SortedCount,2);
	LCDWriteStringXY(12,0,"PS");
	LCDWriteIntXY(12,1,(Count_OptOR-Count_OptEX),2);
	return(0);

}
// IO configuration
void configIO(){
	/* IO Ports Definition */
	DDRA = 0xFF;	// PORTA output, Stepper motor drive
	DDRB = 0xFF;	// PORTB output, DC motor drive
	DDRC = 0xFF;	// PORTC output, LEDs debugging
	DDRD = 0x00;	// PORTD[0,3] input, interrupts
	DDRE = 0x00;	// PORTE[4,7] input, interrupts
	DDRF = 0x00;	// PORTF1 Reflective ADC interrupt
}

// PWM configuration for DC motor
void configPWM(int duty_cyc){
	TCCR0A |= _BV(WGM01) | _BV(WGM00) | _BV(COM0A1); //Set Timer0 to Fast PWM and clear OC0A on Compare match
	TCCR0B |= _BV(CS01);	//Setting prescale to a frequency of 3.9 kHz
	OCR0A = duty_cyc;
}

// ADC configuration
void configADC(){
	ADCSRA |= _BV(ADEN); // enable adc
	ADCSRA |= _BV(ADIE); // enable interrupt of adc
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS0)); // adc scaler division factor 32
	// set 10bit ADC value structure, ADCH[7,0] = ADC[9,2], ADCL[7,6] = ADC[1,0]
	ADMUX |= _BV(REFS0); // Vcc 3.3v Voltage reference with external capacitor on AREF pin
	ADMUX |= _BV(MUX0); // channel select, ADC1
}

// Interrupt configuration
void configInterrupts(){
	EIMSK |= 0b00011111;					// Enable INT0-6					
	// Rising edge: INT2 & INT 6
	EICRA |= (_BV(ISC20) | _BV(ISC21));		// INT2 rising edge
	EICRB |= (_BV(ISC60) | _BV(ISC61));		// INT6 rising edge
	// Falling edge: INT0,1,3,4,5
	EICRA |= _BV(ISC01);					// INT0 falling edge
	EICRA |= _BV(ISC11);					// INT1 falling edge
	EICRA |= _BV(ISC31);					// INT3 falling edge
	EICRB |= _BV(ISC41);					// INT4 falling edge
	EICRB |= _BV(ISC51);					// INT5 falling edge
}

//DC Motor
void DCMotorCtrl(int sysSTATE){
	switch (sysSTATE){
		case 0:
		PORTB = DC_ccw;
		break;
		case 1:
		PORTB = DC_Hstop;
		break;
	}
}

void ADC_calibrate(){
	
	for(int j=0;j<4;j++){ //4 total trials, one for each category
		int cal_min = 0xFFFF;
		for(int i=0;i<10;i++){// run part through 10 times
			
			while(itemready == 0);

			LCDWriteIntXY(0,0,ADC_min,3);
			//mTimer(200);
			//LCDClear();
			if(ADC_min < cal_min){
				cal_min = ADC_min;
				LCDWriteIntXY(12,1,cal_min,3);
			}
			ADC_min = 0xFFFF;
			itemready = 0;
		}
		LCDClear();
	}
}

// stepper go home
void stepper_Home(){
	while(Hall_Flag == 0){
		stepperRotate(1,0);
	}
	CurrentPosition = 1;
}

void stepperRotate(int steps, int direction) {
	int maxdelay = 25;
	int mindelay = 7;
	//int delays[18]  = {6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,24};
	//int d = 17;
	int delay = maxdelay;
	switch(direction){
		case 0: //clockwise
		//LCDWriteIntXY(0,1,cat,1);
		for(int i=0; i<steps; i++){
			currentStep++;
			if(currentStep>3){
				currentStep = 0;
			}
			PORTA = stepper[currentStep];
			if(i < 18 && delay >= mindelay){
			delay -=1;
			}
			if( (steps-1)<9){
			delay +=2;
			}
			mTimer(delay);
		}
		break;
		case 1: //counter-clockwise
		//LCDWriteIntXY(0,1,cat,1);
		for(int i=0; i<steps; i++){
			if(currentStep == 0){
				currentStep = 3;
			}else{
				currentStep--;
				}
			PORTA = stepper[currentStep];
			if(i < 18 && delay >= mindelay){
			delay -=1;
			}
			if( (steps-1)<9 && delay <= maxdelay){
			delay +=2;
			}
			mTimer(delay);
		}
		break;
	}
}

// stepper_position
int stepperSorting(int CurrentPosition, int DesiredPosition){
	int diff = (DesiredPosition - CurrentPosition);
	if((diff == 1) || (diff == -3)){
		stepperRotate(50, 0);
		//DCMotorCtrl(0);
		//stepperRotate(10,0);
	}
	else if((diff == -1) || (diff == 3)){
		//LCDWriteString("here");
		stepperRotate(50, 1);
		//DCMotorCtrl(0);
		//stepperRotate(10, 0);
	}
	else if((diff == 2) || (diff == -2)){
		stepperRotate(100, 0);
		//DCMotorCtrl(0);
		//stepperRotate(10, 0);
	}
	else{
		stepperRotate(0, 0);
	}
	CurrentPosition = DesiredPosition;

	return CurrentPosition;
}


ISR(ADC_vect){					// ADC interrupt conversion , PF1
	if (ADC < ADC_min){
		ADC_min = ADC;
	}
	if ((PIND & 0x04) == 4){
		ADCSRA |= _BV(ADSC);
	}else{
		LCDClear();
		LCDWriteIntXY(0,0,ADC_min,3);
		if(ADC_min <= 200){
			cylin[Count_OptOR-1].category = 2; // Alluminum
			//cat = 2;
		}
		else if(ADC_min <= 700){
			cylin[Count_OptOR-1].category = 0; // Steel
			//cat = 0;
		}
		else if(ADC_min <= 935){
			cylin[Count_OptOR-1].category = 3; // White plastic
			//cat = 3;
		}
		else if(ADC_min <= 1024){
			cylin[Count_OptOR-1].category = 1; // Blk plastic
			//cat = 1;
		}
		else{
			cylin[Count_OptOR-1].category = 4; // Unknown
			//cat = 4;
			//LCDWriteString("unknown");
			//mTimer(2000);
			//LCDClear();
		}
	}
}
	
//Set up External Interrupt 0
ISR(INT0_vect){
	//LCDWriteStringXY(1,0,"EX");
	mTimer(20);
	if((PIND & 0x01) == 0x00){
		
		if (pauseflag == 1){
			pauseflag = 0;
		}
		else{
			pauseflag = 1;
			STATE = 1;
		}
		while((PIND & 0x01) == 0x00);
		mTimer(20);
	}
}
//Set up External Interrupt 1
ISR(INT1_vect){
	mTimer(20);
	while((PINE & 0x02) == 0x00);
	mTimer(20);
	rTimer();
}
	
/* Set up the External Interrupt 2 Vector */
ISR(INT2_vect){
	/* Toggle PORTC bit 2 */
	//LCDWriteStringXY(0,1,"INT2");
	ADC_min = 0xFFFF;
	Count_OptOR++;
	ADCSRA|= _BV(ADSC); // rising on INT2, start ADC conversion
}

ISR(INT3_vect){
	/* Toggle PORTC bit 3 */
	Hall_Flag = 1;
	//STATE = 3;
}

// For press buttom low to stop/resume the belt, PE4, INT4
ISR(INT4_vect){
	Count_OptEX++;
	STATE = 3;
}

//Set up External Interrupt 5
ISR(INT5_vect){
	
	
}

ISR(TIMER3_COMPA_vect){
	//LCDClear();
	//LCDWriteString("Timer Vector");
	//mTimer(2000);
	STATE = 5;
}

//Set up External Interrupt 6
ISR(INT6_vect){
	//LCDWriteString("6");
	//mTimer(2000);
	//LCDClear();
}

// If an unexpected interrupt occurs (interrupt is enabled and no handler is installed,
// which usually indicates a bug), then the default action is to reset the device by jumping
// to the reset vector. You can override this by supplying a function named BADISR_vect which
// should be defined with ISR() as such. (The name BADISR_vect is actually an alias for __vector_default.
// The latter must be used inside assembly code in case <avr/interrupt.h> is not included.
ISR(BADISR_vect)
{
	LCDClear();
	LCDWriteString("BAD ISR");
	mTimer(200);
	LCDClear();
}

void mTimer(int count){
	int i;
	i = 0;
	TCCR1B|=_BV(WGM12)|_BV(CS11);
	OCR1A = 0x03E8;
	TCNT1 = 0x0000;
	//TIMSK1 = TIMSK1|0b00000010;
	TIFR1 |=_BV(OCF1A);
	while(i<count){
		if ((TIFR1 & 0x02) == 0x02){
			TIFR1|=_BV(OCF1A);
			i++;
		}
	}
	return;
}

void rTimer(){ //Rampdown Timer
	TCCR3B|=(_BV(WGM32) | _BV(CS30) | _BV(CS32));
	OCR3A = 0x6000;
	TCNT3 = 0x0000;
	TIMSK3 = TIMSK3|0b00000010;
	TIFR3 |=_BV(OCF3A);
	return;
}
