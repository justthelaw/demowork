#include "U0_LCD_Driver.c"

#ifndef F_CPU

#ifdef USING_BOOTLOADER
#define F_CPU 2000000UL
#else
#define F_CPU 16000000UL
#endif

#endif

#define pwm_coma1 5
#define pwm_coma0 4

#define pwm_wgm0 6
#define pwm_wgm1 3

#define pwm_cs0 0
#define pwm_cs1 1
#define pwm_cs2 2

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void SetupInterrupts(void);
void BootLoaderFixes(void);
void pwmInit(void);
void ADC_init(void);
int ADC_read(void);

void followTheLight(void);
void avoidTheLight(void);
void fullSweep(void);
void localSweep(int prim_angle);
void displayAngle(int angle);

volatile uint8_t adclVal, adchval = 0; //Plan to use these in interrupt

int FULL_SWEEP_MAX[9] = {0};
int LCL_SWEEP_MAX[3] = {0};
int PRIM_ANGLE[3] = {0};

int main(){
	
	//Allows interrupts in debug mode
	#ifdef USING_BOOTLOADER
	BootLoaderFixes();
	#endif
		
	//Setup All Pushbuttons
	DDRB |= 1 << PINB1;
	DDRB  &= ~0b11000000;  //set B6,B7 as inputs, B1 is an output
	PORTB |=  0b11010000;  //enable pull up resistors on B4,B6,B7
	
	LCD_Init(); //Initialize LCD
	SetupInterrupts();	//setup the interrupts
	pwmInit();
	ADC_init();
	sei();				//enable global interrupts
	
	while (1){
		LCD_AllSegments(FALSE); //clears screen
			LCD_WriteDigit('P', 0);
			LCD_WriteDigit('R', 1);
			LCD_WriteDigit('O', 2);
			LCD_WriteDigit('J', 3);
			LCD_WriteDigit(' ', 4);
			LCD_WriteDigit('4', 5);
		_delay_ms(1000);
	}
}

//called for PB6, PB7
ISR(PCINT1_vect) 		//remember this is called on pin change 0->1 and 1->0
{
	static uint8_t p6Prev=1; //for storing previous value of PB6 to detect
	static uint8_t p7Prev=1; //for storing previous value of PB7 to detect
	
	
	if( !((PINB & (1<<6)) ) && ((p6Prev & (1<<6))) ) //when on UP button status being newly pressed, but not when it is released
	{
		followTheLight();
	}
	else if( !((PINB & (1<<7)) ) && ((p7Prev & (1<<7))) ) //when on DOWN button status being newly pressed, but not when it is released
	{
		avoidTheLight();
	}

	p6Prev = (PINB); //save UP button status
	p7Prev = (PINB); //save DOWN button status
}

void SetupInterrupts(void)
{
	//Setup for Center Button Interrupt
	PCMSK1  |= (1<<PCINT14); //Unmask bit for UP Button on Butterfly, PB6->PCINT14 to allow it to trigger interrupt flag PCIF1
	PCMSK1  |= (1<<PCINT15); //Unmask bit for DOWN Button on Butterfly, PB7->PCINT15 to allow it to trigger interrupt flag PCIF1
	EIMSK   = (1<<PCIE0) | (1<<PCIE1);    //Enable interrupt for flag PCIF1 and PCIF0
	//Enable interrupts from PCIE0
}

//This performs adjustments needed to undo actions of Butterfly boot loader
void BootLoaderFixes(void)
{
	//Boot loader Disables E2 and E3 digital input drivers, which are used for left and right
	//The following code re-enables them by clearing the appropriate disable bits
	DIDR1 &= ~((1<<AIN0D)|(1<<AIN1D));
}


//Input: none
//Output: none
//Initiates the 8-bit PWM
void pwmInit()
{
	//The way this generates a PWM is whenever the counter is equal to OCR0A (you set this number), it sends a pulse through PINB4
	
	//TCCR0A is the timer control register
	TCCR0A |= (1<<WGM00)|(0<<WGM01)|(1<<COM0A1)|(0<<COM0A0)|(0<<CS02)|(0<<CS01)|(1<<CS00); //Phase Correcting PWM
	
	
	//Here it selects a clk that is 1/16 the speed as processor speed
	CLKPR |= (0<<CLKPS3)|(1<<CLKPS2)|(1<<CLKPS2)|(0<<CLKPS0);
	
	TIMSK0 |= (0<<OCIE0A)|(0<<TOIE0);//Interrupt triggered when PWM is pulsed
	//Effectively controls interrupts by the timer, refer to Section 13.9.4
	
	// 120Hz = 16Mhz / ( 16 *( 1 + TOP ) ) -> TOP = 8332
	ICR1 = 8332;
	
	DDRB |= 1<<PINB4; //output pin set again just in case
	
}

//input: None
//output: none
//Setups the ADC for reading
void ADC_init()
{
	//Refer to Section 21 in the datasheet for info on how ADC operates
	
    /* Setup ADC to use int 1.1V reference 
    and select LDR sensor channel */
    ADMUX = (1<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0<<MUX3) | (0<<MUX2) | (1<<MUX1) | (0<<MUX0);
	//MUX3 - MUX0 are used to select the ADC channels
	//REFS1, REFS0 are used to select the reference voltage. In this case, we have selected the internal 1.1v

    /* Set conversion time to 
    //112usec = [(1/(8Mhz / 64)) * (14 ADC clocks  per conversion)]
     and enable the ADC*/
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN) | (0<<ADIE) | (1<<ADATE);
	
	/* Perform Dummy Conversion to complete ADC init */
	ADCSRA |= (1<<ADSC);
	//Turning on ADSC starts a conversion, it turns off when it completes a conversion
	
	return;
}

//input: None
//output: none
// follows the steps to produce follow the light mode
void followTheLight(){
	int angle = 0, offset = 0, largest_val = 0;
	
	// announce to user what mode it is
	LCD_AllSegments(0);
	LCD_WriteDigit('F', 0);
	LCD_WriteDigit('T', 1);
	LCD_WriteDigit('L', 2);
	
	cli(); // disables all interrupts as document states
	fullSweep(); // go through the full 180 degree motion
	sei(); // re-enables interrupts
	
	// loop to find largest ADC value from the LDR
	for (int i = 0; i < 9; i++){
		if (!i)
			largest_val = FULL_SWEEP_MAX[i];
		if (FULL_SWEEP_MAX[i] > largest_val){
			angle = i * 20;
			largest_val = FULL_SWEEP_MAX[i];
		}
	}
	
	// shows user which angle had the most light
	displayAngle(angle);
	_delay_ms(800);
	
	LCD_AllSegments(0);
	LCD_WriteDigit('B', 0);
	LCD_WriteDigit('A', 1);
	LCD_WriteDigit('C', 2);
	LCD_WriteDigit('K', 3);
	
	// since I can not get my servo to go clockwise, I reset it's position to 0 degrees	
	for (int j = 10; j <= 100; j += 10){
		OCR0A = j;
		_delay_ms(25);
	}
	OCR0A = 0;
	
	LCD_AllSegments(0);
	LCD_WriteDigit('F', 0);
	LCD_WriteDigit('T', 1);
	LCD_WriteDigit('L', 2);
	_delay_ms(300);
	
	
	localSweep(angle); // goes from -10 to +10 range in 10 degree increments
	
	// loop to find largest ADC value from the LDR
	for (int i = 0; i < 3; i++){
		if (LCL_SWEEP_MAX[i] > FULL_SWEEP_MAX[angle / 20])
			offset = i;
	}
	// switch statement alters the found angle based on the the findings
	switch(offset){
		case 0:
			if (angle)
				angle -= 10;
			break;
		case 1:
			break;
		case 2:
			if (angle < 180)
				angle += 10;
			break;
	}
	// displays the new angle with the most light to the user
	displayAngle(angle);
	_delay_ms(800);
}

//input: None
//output: none
// follows the steps to produce follow the light mode
void avoidTheLight(){
	int smallest_val = 0, offset = 0, angle = 0;
	
	// announce to user what mode it is
	LCD_AllSegments(0);
	LCD_WriteDigit('A', 0);
	LCD_WriteDigit('T', 1);
	LCD_WriteDigit('L', 2);
	
	cli(); // disables all interrupts as document states
	fullSweep(); // go through the full 180 degree motion
	sei(); // re-enables interrupts
	
	// loop to find smallest ADC value from the LDR
	for (int i = 0; i < 9; i++){
		if (!i)
			smallest_val = FULL_SWEEP_MAX[i];
		if (FULL_SWEEP_MAX[i] < smallest_val){
			angle = i * 20;
			smallest_val = FULL_SWEEP_MAX[i];
		}
	}
	
	// displays angle with least light to user
	displayAngle(angle);
	_delay_ms(800);
	
	LCD_AllSegments(0);
	LCD_WriteDigit('B', 0);
	LCD_WriteDigit('A', 1);
	LCD_WriteDigit('C', 2);
	LCD_WriteDigit('K', 3);
	
	// loop to find smallest ADC value from the LDR
	for (int j = 10; j <= 100; j += 10){
		OCR0A = j;
		_delay_ms(25);
	}
	OCR0A = 0;
	
	LCD_AllSegments(0);
	LCD_WriteDigit('A', 0);
	LCD_WriteDigit('T', 1);
	LCD_WriteDigit('L', 2);
	_delay_ms(300);	
	
	localSweep(angle); // goes from -10 to +10 range in 10 degree increments
	
	for (int i = 0; i < 3; i++){
		if (LCL_SWEEP_MAX[i] < FULL_SWEEP_MAX[angle / 20])
		offset = i;
	}
	// switch statement alters the found angle based on the the findings
	switch(offset){
		case 0:
			if (angle)
				angle -= 10;
			break;
		case 1:
			break;
		case 2:
			if (angle < 180)
				angle += 10;
			break;
	}
	// displays final angle finding to the user
	displayAngle(angle);
	_delay_ms(800);
}

//input: None
//output: none
// rotates the servo 180 degrees from where it started
void fullSweep(){
	for (int j = 10; j <= 100; j += 10){
		OCR0A = j;
		_delay_ms(25); //Needs enough time so the counter doesn't get overwhelmed
		FULL_SWEEP_MAX[j-10] = ADC_read(); // stores the LDR value to it's array index
	}
	OCR0A = 0; //Stop the PWM signal
}
void localSweep(int prim_angle){
	int secondary_angle;
	
	// here I move the servo to the position 10 degrees less then the primary angle, if prim_angle != 0 degrees
	if (prim_angle){
		for (int j = 10; j <= prim_angle - 10; j += 10){
			OCR0A = j;
			_delay_ms(13);
		}
	}
	// if not an edge case
	if ( prim_angle < 180 && prim_angle > 0 ){
		for (int j = 10; j <= 40; j += 10){
			OCR0A = j;
			_delay_ms(13); //Needs enough time so the counter doesn't get overwhelmed
			LCL_SWEEP_MAX[j-10] = ADC_read();
		}
		secondary_angle = 30;
	}
	else if (prim_angle == 0){
		for (int j = 10; j <= 30; j += 10){
			OCR0A = j;
			_delay_ms(13); //Needs enough time so the counter doesn't get overwhelmed
			LCL_SWEEP_MAX[j] = ADC_read();
			LCL_SWEEP_MAX[0] = 0;
		}
		secondary_angle = 20;
	}
	else{
		for (int j = 10; j <= 30; j += 10){
				OCR0A = j;
				_delay_ms(12); //Needs enough time so the counter doesn't get overwhelmed
				LCL_SWEEP_MAX[j-10] = ADC_read();
			LCL_SWEEP_MAX[2] = 0;
		}
		secondary_angle = 20;
	}
	OCR0A = 0; //Stop the PWM signal
	LCL_SWEEP_MAX[0] = 0;
	_delay_ms(120);
	// reset the the servo to original angle
	for (int j = 10; j <= 180 - prim_angle - secondary_angle; j += 10){
		OCR0A = j;
		_delay_ms(12);
	}
	OCR0A = 0;
}

//input: integer number of the angle to display
//output: none
// rotates the servo 180 degrees from where it started
void displayAngle(int angle){
	if (angle >= 100){
		PRIM_ANGLE[2] = 1;
		angle -= 100;
	}
	else
	PRIM_ANGLE[2] = 0;
	PRIM_ANGLE[1] = angle / 10;
	PRIM_ANGLE[0] = 0;
	
	LCD_AllSegments(0);
	if (PRIM_ANGLE[2])
		LCD_WriteDigit(PRIM_ANGLE[2] + 48, 0);
	LCD_WriteDigit(PRIM_ANGLE[1] + 48, 1);
	LCD_WriteDigit(PRIM_ANGLE[0] + 48, 2);
}

//input: none
//output: int - returns the ADC value after reading
//reads and gets the full 10-bits of ADC
int ADC_read()
{
	int ldrLow_ADCR; //LDR value to hold ADCL
	int ldrHigh_ADCR; //LDR value to hold ADCH
	
	ADCSRA |= (1 << ADSC); //this is automatically cleared
	ldrLow_ADCR = ADCL;
	ldrHigh_ADCR = ADCH;
	
	return (ldrHigh_ADCR << 8 | ldrLow_ADCR); //Shifts bits from ADCH left 8 times, then combines with ADCL to create full 10 bit result
}