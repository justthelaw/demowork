#ifndef F_CPU

#ifdef USING_BOOTLOADER
#define F_CPU 2000000UL
#else
#define F_CPU 16000000UL
#endif

#endif


#include <util/delay.h>
#define PORTB5_SPEAKER_MASK 0b00100000

#define FREQ_G4_HZ 392
#define HALFPERIOD_G4_US 1276 //delay in microseconds

#define NUMBER_OF_SONGS 10
#define MAX_SONG_LENGTH 8
#define USER_LINE_MAX 16
#define NOTE_A 0
#define NOTE_B 1
#define NOTE_C 2
#define NOTE_D 3
#define NOTE_E 4
#define NOTE_F 5
#define NOTE_G 6
#define NOTE_R 7
#define C4 261.63
#define D4 293.66
#define E4 329.63
#define F4 349.23
#define G4 392
#define A4 440
#define B4 499.16
#define del_C4 191
#define del_D4 170
#define del_E4 151
#define del_F4 143
#define del_G4 127
#define del_A4 113
#define del_B4 100
#define duration 4

void playNote(int pos, int secret);

void playNote(int pos, int secret){
    int lowend, highend, truepos, divisor = 0;
    truepos = pos - 90;
   
    lowend = (secret - 90 ) / 7;
    highend = (270 - secret) / 7;

    //PORTB |= (1 << 5);
	// corresponding arrays for frequency and array values from .h file
	int freq[7] = {C4, D4, E4, F4, G4, A4, B4};
	int delay[7] = {del_C4, del_D4, del_E4, del_F4, del_G4, del_A4, del_B4};

    if (pos > secret){
        divisor = (pos - secret)/highend;
        // lazy way to flip the scoring system for the pitch of the note
        switch (divisor)
            {
            case 0:
                divisor = 7;
                break;
            case 1:
                divisor = 6;
                break;
            case 2:
                divisor = 5;
                break;
            case 3:
                divisor = 4;
                break;
            case 4:
                divisor = 3;
                break;
            case 5:
                divisor = 2;
                break;
            case 6:
                divisor = 1;
                break;
            case 7:
                divisor = 0;
                break;
			default: 
				divisor = 0;
            }
        }
    else
        // assigns value if lower than secret
        divisor = truepos/lowend;

    // this is my method from project 2 to play the note wanted
    for (int i = 0; i < floor(freq[divisor] / 4) * duration; i++){
        PORTB |= (1 << 5);                    //Activate buzzer on PINB5
        for (int n = 0; n < delay[divisor]; n++) //delays for a variable amount of time determined by 'i'
        {
            _delay_ms(0.001);
        }
        PORTB &= ~(1 << 5);                    //Deactivate buzzer
        for (int n = 0; n < delay[divisor]; n++) //delays for a variable amount of time determined by 'i'
        {
            _delay_ms(0.001);
        }
    }

    //DDRB &= ~(1 << 5);
    return;
}