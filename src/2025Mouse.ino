/*
 * ATTiny85_Internal_8MHz_Template.cpp
 *
 * Created: 11/5/2025
 * Charlieplexing functions modeled after the code found here:
 *   http://www.instructables.com/id/CharliePlexed-LED-string-for-the-Arduino/step7/Coding-the-Arduino/
 * 2019 - Added prescalar of 2 to accomodate 8MHz clock div/8 (low fuse 0x62)
 */ 

#ifdef F_CPU  
#undef F_CPU
#endif	
#define F_CPU 4000000 // 4 MHz

// ATMEL ATTINY85 / ARDUINO
//	            						  ____
//  		   (D5/A0) RST/PB5	1|*	  |8  VCC
//	 PWM   (D3/A3)	   PB3	2|	  |7  PB2  (D2/A1)
//	 PWM   (D4/A2)	   PB4	3|	  |6  PB1  (D1)		PWM
//	        				   GND	4|____|5  PB0  (D0)		PWM

#include <avr/pgmspace.h>
// Power Mgt:
#include <avr/sleep.h>
#include <avr/wdt.h>

// FUNCTION PROTOTYPES
void displayChar(int from, int through, int tranSpeed); // loads a Pattern from above. From & through specify what patterns to show. tranSpeed is the speed of the frames
//void ledSpecify(int highPin, int lowPin); // This allows you to manually control which pin goes low & which one goes high
void turnon(int led); // This turns on a certian led from the list of leds
void alloff(); // This turns all the LED's off
void setup_watchdog(int ii);
void system_sleep();
void twinkle(int tranSpeed, int framesOff, int seqCount);

// ******************************************************************************************************************
//
// Much of the charlieplex code has been modified from http://www.instructables.com/id/CharliePlexed-LED-string-for-the-Arduino/step7/Coding-the-Arduino/
// The power management code is from 

#define LEDCount 10
byte pins[4];
//const byte LEDCount = 20;

// The attiny85 watchdog only sleeps for 8 seconds at a time, so for a longer period of time we need to setup a counter variable
//	 and check that against the # of times it wakes up.
// The internal clock isn't very accurate, so you need to factor that in. My tests show a deviation of ~0.9198 at 900 seconds,
//	 so a rough equation would be [ SleepTime = DesiredTime * 0.9198 ]
// Every MCU will be different, so some experimentation will be necessary if you want accurate intervals
// Some example times:
//	 5:00 -> 276
//	10:00 -> 552
//	15:00 -> 828
//	30:00 -> 1656
int SleepTime = 416; // Sleep time in seconds

int SleepCnt = SleepTime/8; // counter for watchdog wake timer

// blinkdelay is used by displayChar to set the microseconds the LED will be on, then off. It can affect brightness, but mostly affects speed
// blinkdelay could be replaced with a timeOn and timeOff to facilitate a type of PWM to increase LED brightness at low voltage 
//		(using a small Rs at higher voltage would be bad for LED unless turn-on time can be limited with this)
int blinkdelay = 90;   //Speed Param 1: smaller = faster
// runSpeed is the number of times displayChar will loop through each pattern before going to the next. Again, this affects speed
byte runSpeed = 90;		 //Speed Param 2: smaller = faster (0-255) - this can be set individually when calling displayChar()

// Random number
int RndNumber = 0;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
volatile boolean f_wdt = 1;

const byte LEDConnections[LEDCount][2] ={	// This stores which outputs the LEDs are connected to: { Anode, Cathode }
	// Bottom Row
	{ 0 , 1 },	  //X1
	{ 1 , 2 },	  //X3
	{ 2 , 3 },	  //X5
	// Row 1
	{ 1 , 0 },	  //X2			  
	{ 2 , 1 },	  //X4
	{ 3 , 2 },	  //X6
	// Row 2
	{ 0 , 2 },	  //X9
	{ 1 , 3 },	  //X11
	// Row 3
	{ 2 , 0 },	  //X10
	{ 3 , 1 },	  //X12
	// Row 4
	//{ 0 , 3 },	  //X15
	//{ 3 , 0 }	  //X7
};

const byte displays[][LEDCount] PROGMEM ={	// This stores the array in Flash ROM.
	// All Off (0)
	{0,0,0,0,0,0,0,0,0,0},

	// Pattern 1: All LEDs Flash (1-2)
	{1,1,1,1,1,1,1,1,1,1},
	{0,0,0,0,0,0,0,0,0,0},

  // Pattern 2: Outside->Inside Fill (3-17)
  {0,0,1,0,0,0,0,0,0,0}, // 3
  {0,0,1,0,0,1,0,0,0,0},
  {0,0,1,0,0,1,0,0,1,0},
  {0,0,1,0,0,1,1,0,1,0},
  {1,0,1,0,0,1,1,0,1,1},
  {1,0,1,0,1,1,1,1,1,1},
  {1,1,1,0,1,1,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1},
  {1,1,1,1,1,0,1,1,1,1},
  {1,1,1,1,1,0,1,1,0,1},
  {1,1,1,1,1,0,0,1,0,1},
  {0,1,1,1,1,0,0,1,0,0},
  {0,1,1,1,0,0,0,0,0,0},
  {0,0,1,1,0,0,0,0,0,0},
  {0,0,1,0,0,0,0,0,0,0}, // 17

  // Pattern 3: (18-26)
  {0,0,1,0,0,0,0,0,0,0}, // 18
  {0,0,1,0,0,1,0,0,1,0},
  {0,0,1,0,0,1,1,0,1,0},
  {1,0,1,0,0,1,1,0,1,1},
  {1,0,1,0,1,1,1,1,1,1},
  {1,0,1,0,1,0,1,1,0,1},
  {1,0,1,0,1,0,0,1,0,1},
  {0,0,1,0,1,0,0,1,0,0},
  {0,0,1,0,0,0,0,0,0,0}, // 26

  // Pattern 4: Starburst (27-37)
  {0,0,1,0,0,0,0,0,0,0}, // 27
  {0,0,1,1,0,0,0,0,0,0},
  {1,0,1,1,0,0,0,1,0,0},
  {1,0,1,1,1,0,1,1,0,0},
  {1,0,1,1,1,0,1,1,1,1},
  {1,1,1,1,1,1,1,1,1,1},
  {1,1,1,0,1,1,1,1,1,1},
  {1,1,1,0,0,1,0,1,1,1},
  {1,1,1,0,0,1,0,0,0,1},
  {1,0,1,0,0,0,0,0,0,1},
  {0,0,1,0,0,0,0,0,0,0}, // 37

};

void setup()   {
	// Set prescalar so the clock runs at 4MHz
	// See here: https://forum.arduino.cc/index.php?topic=624890.0
	cli(); //disable interrupts - this is a timed sequence.
	CLKPR=0x80; //change enable
	CLKPR=0x01; //set prescaler to 2
	sei(); //enable interrupts
  
	// initialize the digital pin as an output:
	pins[0] = 0;
	pins[1] = 1;
	pins[2] = 2;
	pins[3] = 3;

	// Status LED
	//pinMode(4, OUTPUT);

	setup_watchdog(9); // Watchdog timer; approximately 8 seconds sleep
	randomSeed(analogRead(A2));	 // Seeds the random number generator for more randomness. A2 must be disconnected!
}

void loop()						
{
	int PastRnd = 0;
	if (f_wdt==1) {	 // wait for timed out watchdog / flag is set when a watchdog timeout occurs

		f_wdt=0;	   // reset flag
		if (SleepCnt >= (SleepTime/8)) {
			SleepCnt = 0; // Reset sleep counter
			// Code to execute after we wake from sleep:		  
			// displayChar( from, through, tranSpeed )
			for (int demos=0; demos<3; demos++) {
				// Generate a random number for pattern selection
				while (RndNumber == PastRnd) { RndNumber = random(1,7); }
        //RndNumber = 4;
				PastRnd = RndNumber;
				switch (RndNumber) {
				case 1: // Pattern 1 - Blink them all  
					for (int loop=0; loop<4; loop++) {
						displayChar(0, 0, 210);
						delay(50);
						displayChar(1, 1, 210); 
						delay(150);
					}
					break;	   
				case 2: // Pattern 2  
          for (int loop=0; loop<3; loop++) {
            displayChar(3, 17, 70);
          }
          //displayChar(47, 47, 70);
					break;
				case 3: // Pattern 3  
          for (int loop=0; loop<3; loop++) {
            displayChar(18, 26, 70);
          }
          //displayChar(47, 47, 70);
					break;
				case 4: // Pattern 4  
          for (int loop=0; loop<3; loop++) {
            displayChar(27, 37, 70);
          }
					break;									  
        case 5:
				case 6: // Twinkle pattern - both 5 & 6 to give it a better chance of being selected
					//twinkle( tranSpeed, framesOff, seqCount)
					twinkle(130,30,15);
					demos=3; // end the display
					break;
				}
			}
		  // End of wake code

			//pinMode(0,INPUT); // Normally you would set the pinMode to INPUT to save power, but since the charlieplexing code does this, we don't need to here
			system_sleep();
			//pinMode(0,OUTPUT); // Set the pinmode back to OUTPUT, as it was before	
		} else {
			SleepCnt++;
			//pinMode(0,INPUT); // Normally you would set the pinMode to INPUT to save power, but since the charlieplexing code does this, we don't need to here
			system_sleep();
			//pinMode(0,OUTPUT); // Set the pinmode back to OUTPUT, as it was before
		}
	}	
}

// ***************************** Functions *****************************

// Twinkle effect - Turns all LEDs on and randomly shuts 1 LED off for a short duration
void twinkle(int tranSpeed, int framesOff, int seqCount)
{
  boolean run = true;
	byte arr[LEDCount];
  byte k;
	int runCnt = 0;
	int LASTindex; // ensures it doesn't select the same LED over and over
	int flickerCount = 1; // # of LEDs to flicker at the same time

  while(run == true) 
  {
		for(int i = 0; i < LEDCount; i++) { arr[i] = 1; } // Reset all to 1
		// Randomly pick flickerCount elements and set to 0 (excluding 3rd element)
		int flickered = 0;
		while (flickered < flickerCount) {
			int index = random(0, 10);
			if (index != 2 && index != LASTindex && arr[index] == 1) { // exclude 3rd element (index 2)
				arr[index] = 0;
			  LASTindex=index;
				flickered++;
			}
		}
		// Turn twinkling LEDs off
    for(int i = 0; i < framesOff; i++) // run tranSpeed times
    {
      for(int j = 0; j < LEDCount; j++) // loop through each line of elements in the byte array
      {
        k = arr[j]; // read each element of the array
        if (k == 2)
        {
          run = false;
        }
        else if(k == 1)
        {
          turnon(j);
          delayMicroseconds(blinkdelay);
          alloff();
        }
        else if(k == 0)
        {
          delayMicroseconds(blinkdelay);
        }
      }
    }
		// Turn all LEDs back on
		for(int i = 0; i < LEDCount; i++) { arr[i] = 1; } // Reset all to 1
    for(int i = 0; i < (tranSpeed-framesOff); i++) // run (tranSpeed-framesOff) times
    {
      for(int j = 0; j < LEDCount; j++) // loop through each line of elements in the byte array
      {
        k = arr[j]; // read each element of the array
        if (k == 2)
        {
          run = false;
        }
        else if(k == 1)
        {
          turnon(j);
          delayMicroseconds(blinkdelay);
          alloff();
        }
        else if(k == 0)
        {
          delayMicroseconds(blinkdelay);
        }
      }
    }
		// If seqCount frames have been presented, exit pattern
		if (runCnt < seqCount )
		{
			runCnt++;
		} else {
	  	return;
		}
  }
}

// Turns specified LED on
void turnon(int led)
{
//	int pospin = LEDConnections[led][0] + 2;  //  <-- what is the +2 for?
//	int negpin = LEDConnections[led][1] + 2;
  int anode = LEDConnections[led][0];
  int cathode = LEDConnections[led][1];
  pinMode (anode, OUTPUT);
  digitalWrite (anode, HIGH);
  pinMode (cathode, OUTPUT);
  digitalWrite (cathode, LOW);
}

// Turns ALL LEDs off
void alloff()
// TM - added sizeof(pins) to determine pin count (instead of hard-coded counts)
{
  for(int i = 0; i < sizeof(pins); i++)
  {
	pinMode (pins[i], INPUT);
  }
}

// Loads a Pattern from byte array displays[][]. From & through specify what patterns to show. tranSpeed is the speed of the frames (smaller=faster)
// Courtesy of "computergeek"
// http://www.instructables.com/id/CharliePlexed-LED-string-for-the-Arduino/step7/Coding-the-Arduino/
void displayChar(int from, int through, int tranSpeed)
{
  boolean run = true;
  byte k;
  int t = from;
  while(run == true)
  {
	for(int i = 0; i < tranSpeed; i++)
	{
	  for(int j = 0; j < LEDCount; j++)
	  {
		  k = pgm_read_byte(&(displays[t][j]));
		  if (k == 2)
		  {
		    run = false;
		  }
		  else if(k == 1)
		  {
		    turnon(j);
		    delayMicroseconds(blinkdelay);
		    alloff();
		  }
		  else if(k == 0)
		  {
		    delayMicroseconds(blinkdelay);
		  }
	  }
	}
	if(through == t){
	  return;
	}
	t++;
  }
}

// Power Management
// *********************************************************************************************

// set system into the sleep state
// system wakes up when watchdog is timed out
void system_sleep() {
	cbi(ADCSRA,ADEN);					 // switch Analog to Digitalconverter OFF

	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();

	sleep_mode();						 // System sleeps here

	sleep_disable();					 // System continues execution here when watchdog timed out
	sbi(ADCSRA,ADEN);					 // switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
	byte bb;
	int ww;
	if (ii > 9 ) ii=9;
	bb=ii & 7;
	if (ii > 7) bb|= (1<<5);
	bb|= (1<<WDCE);
	ww=bb;

	MCUSR &= ~(1<<WDRF);
	// start timed sequence
	WDTCR |= (1<<WDCE) | (1<<WDE);
	// set new watchdog timeout value
	WDTCR = bb;
	WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
	f_wdt=1;  // set global flag
}
