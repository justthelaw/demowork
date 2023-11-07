
#ifndef F_CPU

#ifdef USING_BOOTLOADER
#define F_CPU 2000000UL
#else
#define F_CPU 16000000UL
#endif

#endif

#include "U0_LCD_Driver.c"
#include "U0_Buzzer_Driver.c"

#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void SetupInterrupts(void);
void BootLoaderFixes(void);
void pwmInit(void);
void welcome(void);
void chooseLock(void);
void mainMenu(void);
void playLock(void);


int CURRENT_STATE, LOCK_SEL, SERV_POS ;

int main(){
	
	//Allows interrupts in debug mode
	#ifdef USING_BOOTLOADER
	BootLoaderFixes();
	#endif

	LOCK_SEL = 0;
	SERV_POS = 90;

	//Setup All Pushbuttons
	DDRB |= (1 << 1);
	DDRB |= (1 << 5);
	DDRB |= (1 << 2);
	DDRB  &= ~0b11010000;  //set B6,B7, B4 as inputs
	PORTB |=  0b11010000;  //enable pull up resistors on B4,B6,B7
	DDRE  &= ~0b00001100;  //set E2,E3 as inputs,
	PORTE |=  0b00001100;  //enable pull up resistors on E2,E3
	
	// timer setup
	TCCR0A = (1<<CS00) | (1<<CS02);
	TCCR1B = (1<<CS10);

	//Initialize LCD
	LCD_Init();
	//setup the interrupts
	SetupInterrupts();	
	//playWelcome();
	welcome();
	//enable global interrupts
	sei();
	mainMenu();
	while(1){
		chooseLock();
	}
}

//called for PB6, PB7
ISR(PCINT1_vect)
{
	//for storing previous value of PB6 to detect
	static uint8_t p4Prev=1; 
	//static uint8_t p6Prev=1; //for storing previous value of PB6 to detect
	//static uint8_t p7Prev=1; //for storing previous value of PB7 to detect
	//         int time_count = 0;

	//when on CENTER button status being newly pressed, but not when it is released
	// This if/else block checks the global CURRENT_STATE variable and acts differently according to the state of the variable
	if( !((PINB & (1<<4)) ) && ((p4Prev & (1<<4))) ) 
	{
		LCD_AllSegments(0); //clears screen
		// just in case, always reset the LED pin and server starting position
		if (!CURRENT_STATE){
			CURRENT_STATE = 1;
			chooseLock();
		}
		else if (CURRENT_STATE == 1){
			CURRENT_STATE = 2;
			playLock();
		}
		else if (CURRENT_STATE == 2) {
			while (SERV_POS != 90)
			{
				PORTB |= (1 << 2);
				_delay_ms(1.5);
				PORTB &= ~(1 << 2);
				_delay_ms(7.5);
				SERV_POS -= 5;
			}
			CURRENT_STATE = 1;
		}
	}
	p4Prev = (PINB); //save CENTER button status

}

//called for PE2, PE3
ISR(PCINT0_vect) //remember this is called on pin change 0->1 and 1->0
{
	static uint8_t p2Prev=1; //for storing previous value of PE2 to detect
	static uint8_t p3Prev=1; //for storing previous value of PE3 to detect
	
	//when on LEFT button status being newly pressed, but not when it is released
	// For left/right, the global var LOCK_SEL will either increment/decrement single int
	if( !((PINE & (1<<2)) ) && ( p2Prev & (1<<2) ) )
	{
		if ( CURRENT_STATE == 1 ){
			switch(LOCK_SEL){
				case 0:
					LOCK_SEL = 2;
					break;
				case 1:
					LOCK_SEL = 0;
					break;
				case 2:
					LOCK_SEL = 1;
					break;
				default:
					LOCK_SEL = 0;
			}
		}
	}
	else if( !((PINE & (1<<3))  ) && ( p3Prev & (1<<3) ) ) //when on RIGHT button status being newly pressed, but not when it is released
	{
		if ( CURRENT_STATE == 1 ){
			switch(LOCK_SEL){
				case 0:
					LOCK_SEL = 1;
					break;
				case 1:
					LOCK_SEL = 2;
					break;
				case 2:
					LOCK_SEL = 0;
					break;
				default:	
					LOCK_SEL = 0;
			}
		}
	}
	
	p2Prev = (PINE); //save LEFT button status
	p3Prev = (PINE); //save DOWN button status
}


void SetupInterrupts(void)
{
	//Setup for Center Button Interrupt
	PCMSK1  |= (1<<PCINT12); //Unmask bit for CENTER Button on Butterfly, PB6->PCINT14 to allow it to trigger interrupt flag PCIF1
	PCMSK0  |= (1<<PCINT2); //Unmask bit for LEFT Button on Butterfly, PE2->PCINT2 to allow it to trigger interrupt flag PCIF0
	PCMSK0  |= (1<<PCINT3); //Unmask bit for RIGHT Button on Butterfly, PE3->PCINT3 to allow it to trigger interrupt flag PCIF0
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

// no interrupts allowed during welcome message
void welcome(){
	int size_of_mess = 9;
	char welcome_mess[9] = {'_', 'W', 'E', 'L','C', 'O', 'M', 'E', '_'};
	
	// Shows "welcome" across the screen both scrolling left and then right
	LCD_AllSegments(0); //clears screen
	for ( int i = 0; i < 7; i++){
			if (i < 4){
				LCD_WriteDigit(welcome_mess[i + 0], 0);
				LCD_WriteDigit(welcome_mess[i + 1], 1);
				LCD_WriteDigit(welcome_mess[i + 2], 2);
				LCD_WriteDigit(welcome_mess[i + 3], 3);
				LCD_WriteDigit(welcome_mess[i + 4], 4);
				LCD_WriteDigit(welcome_mess[i + 5], 5);
				_delay_ms(180);
			}
			else{
				LCD_WriteDigit(welcome_mess[size_of_mess - i - 3], 0);
				LCD_WriteDigit(welcome_mess[size_of_mess - i - 2], 1);
				LCD_WriteDigit(welcome_mess[size_of_mess - i - 1], 2);
				LCD_WriteDigit(welcome_mess[size_of_mess - i + 0], 3);
				LCD_WriteDigit(welcome_mess[size_of_mess - i + 1], 4);
				LCD_WriteDigit(welcome_mess[size_of_mess - i + 2], 5);
				_delay_ms(180);
			}
	}
	_delay_ms(500);
	LCD_AllSegments(0);
	return;
}

// interrupts allowed, waits for user to press "center" button, shows this message until
void mainMenu(){
	sei();
	CURRENT_STATE = 0;
	int size_of_mess = 42;
	char welcome_mess_2[42] = {"__LOCKPICKING_GAME_PRESS_CENTER_TO_START__"};
		
	while(!CURRENT_STATE){
		LCD_AllSegments(0);
		for ( int i = 0; i < 1000; i++){
				LCD_WriteDigit(welcome_mess_2[i % size_of_mess], 0);
				LCD_WriteDigit(welcome_mess_2[(i + 1) % size_of_mess], 1);
				LCD_WriteDigit(welcome_mess_2[(i + 2) % size_of_mess], 2);
				LCD_WriteDigit(welcome_mess_2[(i + 3) % size_of_mess], 3);
				LCD_WriteDigit(welcome_mess_2[(i + 4) % size_of_mess], 4);
				LCD_WriteDigit(welcome_mess_2[(i + 5) % size_of_mess], 5);
				_delay_ms(110);
		}
	}
	return;
}

void chooseLock(){

	CURRENT_STATE = 1;
	sei();				//enable global interrupts
			
	int time_count = 0;
			
	char menu_sel[3][5] = {"LOCK1", "LOCK2", "LOCK3"};
	int size_of_mess = 55;
	char reminder[55] = {"__PRESS_LEFT-RIGHT_TO_SELECT_LOCK_PRESS_CENTER_TO_START"};
	int index = 0;
	int lock_cache = LOCK_SEL;
	
	
	//TCNT0 = 0x00;
	//TIFR0=0x01; //clear timer1 overflow flag

	while (1){
		index = (LOCK_SEL);
		if ( index < 0 ){
			if (index == -2)
			index = 1;
			else if (index == -1)
			index = 2;
			else
			index = 0;
		}
		
		if (LOCK_SEL != lock_cache){
			time_count = 0;
			lock_cache = LOCK_SEL;
		}
		
		//
		if ((TIFR0 & 0x01) != 0){
			TCNT0 = 0x00;
			TIFR0=0x01; //clear timer1 overflow flag
			time_count++;
		}
		// displays current lock the user has currently selected
		if (time_count % 27 == 1){
			LCD_AllSegments(0);
			LCD_WriteDigit(menu_sel[ index ][0], 0);
			LCD_WriteDigit(menu_sel[ index ][1], 1);
			LCD_WriteDigit(menu_sel[ index ][2], 2);
			LCD_WriteDigit(menu_sel[ index ][3], 3);
			LCD_WriteDigit(menu_sel[ index ][4], 4);
		}
		
		// approximately 15 second timeout using timer0. Scrolls through message once, or until interrupted
		if (time_count >= 480){
			LCD_AllSegments(0);
			for ( int i = 0; i < size_of_mess - 5; i++){
				LCD_WriteDigit(reminder[i % size_of_mess], 0);
				LCD_WriteDigit(reminder[(i + 1) % size_of_mess], 1);
				LCD_WriteDigit(reminder[(i + 2) % size_of_mess], 2);
				LCD_WriteDigit(reminder[(i + 3) % size_of_mess], 3);
				LCD_WriteDigit(reminder[(i + 4) % size_of_mess], 4);
				LCD_WriteDigit(reminder[(i + 5) % size_of_mess], 5);
				if (LOCK_SEL != lock_cache)
					break;
				else 
					_delay_ms(110);
			}
			if (LOCK_SEL == lock_cache)
				_delay_ms(800);
			time_count = 0;
		}
		
	}
}

void playLock(){
	CURRENT_STATE = 2;
	int size_of_mess = 81;
	char message[81] = {"__PRESS_UP-DOWN_TO_PICK_LOCK_LED_WILL_TURN_ON_WHEN_CORRECT_PRESS_CENTER TO_QUIT__"};
	int secret_array[3] = {120, 215, 255};
	int lock_picked = 0, counter = 0, index = 0, move_made = 0, last_move = 0;
	
	int time_count = 0;
	int lock_active = LOCK_SEL;

	static uint8_t p6Prev = 1; //for storing previous value of PB6 to detect
	static uint8_t p7Prev=1; //for storing previous value of PB6 to detect
	sei();				//enable global interrupts
	while(!lock_picked){
		CURRENT_STATE = 2;
		move_made = 0;
		// using timer0 instead of delay for timing things
		if ((TIFR0 & 0x01) != 0){
			TCNT0 = 0x00;
			TIFR0=0x01; //clear timer1 overflow flag
			counter++;
		}
		// as long as user is holding button "UP", this will be trigger to move server towards 90 degrees
		if ( !((PINB & (1<<6)) ) && ((p6Prev & (1<<4))) ) {
			while ( !((PINB & (1<<6)) ) && ((p6Prev & (1<<4))) ) {
				//();
				if (SERV_POS > 90){
					if (  (TIFR0 & 0x01) != 0 ){
							SERV_POS -= 5;
							time_count = 0;
							TCNT0 = 0x00;
							TIFR0=0x01; //clear timer1 overflow flag
						
						PORTB |= (1<<2);
						_delay_ms(1.5);
						PORTB &= ~(1<<2);
						_delay_ms(7.5);
					}
				}
				if (secret_array[lock_active] == SERV_POS)
				{
					PORTB |= (1<<1);
					_delay_ms(60);
					PORTB &= ~(1<<1);
				}
				if (!move_made)
					move_made = 1;
				sei();
			}
		}
		// as long as user is holding button "DOWN", this will be trigger to move server towards 270 degrees
		if ( !((PINB & (1<<7)) ) && ((p7Prev & (1<<4))) ){
			while ( !((PINB & (1<<7)) ) && ((p7Prev & (1<<4))) ) {
				//cli();
				if (SERV_POS < 270){
					if (  (TIFR0 & 0x01) != 0  ){
							SERV_POS += 5;
							time_count = 0;
							TCNT0 = 0x00;
							TIFR0=0x01; //clear timer1 overflow flag
					
							PORTB |= (1<<2);
							_delay_ms(2.9);
							PORTB &= ~(1<<2);
							_delay_ms(22);
					}
				}
				if (secret_array[lock_active] == SERV_POS)
				{
					PORTB |= (1<<1);
					_delay_ms(60);
					PORTB &= ~(1<<1);
				}
				if (!move_made)
					move_made = 1;
				sei();
			}
		}
		// displays a scrolling message as long as user isn't holding up or down
		if ( counter >= 8){
			//cli();
			LCD_AllSegments(0);
			LCD_WriteDigit(message[index % size_of_mess], 0);
			LCD_WriteDigit(message[(index + 1) % size_of_mess], 1);
			LCD_WriteDigit(message[(index + 2) % size_of_mess], 2);
			LCD_WriteDigit(message[(index + 3) % size_of_mess], 3);
			LCD_WriteDigit(message[(index + 4) % size_of_mess], 4);
			LCD_WriteDigit(message[(index + 5) % size_of_mess], 5);
			counter = 0;
			index ++;
			sei();
		}
		// if user lands on the lock position and stays there, LED stays lit up and then goes back to main menu
		if (secret_array[lock_active] == SERV_POS){
			cli();
			LCD_AllSegments(0);
			LCD_WriteDigit('W', 0);
			LCD_WriteDigit('I', 1);
			LCD_WriteDigit('N', 2);
			PORTB |= (1<<1);
			_delay_ms(1500);
			PORTB &= ~(1<<1);
			lock_picked = 1;
			move_made = 0;
			while (SERV_POS != 90){
				PORTB |= (1 << 2);
				_delay_ms(1.5);
				PORTB &= ~(1 << 2);
				_delay_ms(7.5);
				SERV_POS -= 5;
			}
			sei();
			CURRENT_STATE = 1;
		}

		if (CURRENT_STATE == 1)
			return;

		cli();
		if (move_made && (abs (secret_array[lock_active] - SERV_POS) < last_move))
			playNote(SERV_POS, secret_array[lock_active]);

		p6Prev = (PINB); //save UP button status
		p7Prev = (PINB); //save DOWN button status

		last_move = abs(secret_array[lock_active] - SERV_POS);
	}
}



