////////////////////////////////////////////////////
////////////////////////////////////////////////////
// AOA EFIS Serial to audio tone 
// Supports: Dynon EFIS data using a Arduino Mega board
// Ver 1.7.4
#define VERSION "174"

// NOTE *********  The Arduino default serial buffer is smaller than the Skyview packet,
// it seems that there could easily be an overflow situation.
// however, testing on 8/18, shows can handle Skyview at 100 msgs per second @ SAMPLE_RATE 3
// Bryan LaPlante 8/17/2026 ---  code review Skyview and handle AOA = "XX" it can send 
// Bryan LaPlante 8/12/2026 ---  testing shows no more lockups.  Updated AOA thresholds for N980RV
// Bryan LaPlante 7/1/2026  ---  still get lockups probably buffer overflows... fix volatile vars...
// Bryan LaPlante 6/21/2026 --- pre-calc the pps vs AOA, to eliminate float point (slow) from main loop
// Bryan LaPlante 6/17/2026 --- increase min speed for tones to 15 (at 5 it is alive standing still!)
// Bryan LaPlante 6/14/2026  --- eliminate dual set points ... (Skyview / D100x)
// Bryan LaPlante 11/27/2025 --- read AOA set points from SD card (D10/100 only for now)
// Bryan LaPlante 11/23/2025 --- convert to Mega (for dual hardware serial ports)
// Bryan LaPlante 10/21/2025 --- conversion to Nano, TimerOne, 


//
// - Read data in from Serial3 on MEGA board. (Dynon is at 115200 baud)
// - Serial is used for debugging to computer IDE. (115200 baud)
// - Audio ouput on pin D11  // BGL
// - If no serial data is detected audio tone turns off and LED1 is off
// - MUTE_AUDIO_UNDER_IAS define is used to only activate audio tones if above a given airspeed
//
// - NOTE: Per https://www.iflyez.com/WAITERS_FLIGHT_DATA_RECORDER.pdf, Dynon Dx Series outputs data over 60 times / sec.
// - That can easily lead to Serial buffer overflow.  Default buffer is 64 bytes



// Christopher Jones 6/10/2016
// 6/10/2016  - Started
// 7/09/2018  - 1.5.1 fix for tone PPS between solid tones.  was not calucating the right ratio for the PPS changes.
// 9/02/2018  - 1.5.2 Atempts to fix hang up issues with skyview data.  Changed start up tones.  added more comments. fix min PPS for low tone.
// 9/28/2018  - 1.6.0 Fixed Skyview lockup issue.
// 9/28/2018  - 1.6.1 allow for automatic efis dectection d100 or skyview.

/* ------------  D10A TEST DATA, send via Serial Monitor with CR+NL termination
00082119+058-00541301200+9141+011-01+15003EA0C701A4   -- from Dynon manual
xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx10xxxxxxxxxx
xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx15xxxxxxxxxx
xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx25xxxxxxxxxx
xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx35xxxxxxxxxx
xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx55xxxxxxxxxx
xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx73xxxxxxxxxx
*/

/* ------------ Dynon Skyview TEST DATA, send via Serial Monitor with CR+NL termination
!1121144703-014+00003310811+01736+003-03+1013-033+110831245+01650023176C  -- from Dynon Install Guide
                                           ^^  13 in this case
ADAHRS data starts with !1, others are !2, etc.
Format is 1 based (! is character 1)
AOA is at 44-45 value up to 99, like D100 series
But can be XX for "not available"
!1121144703-014+00003310811+01736+003-03+1010-033+110831245+01650023176C
!1121144703-014+00003310811+01736+003-03+1015-033+110831245+01650023176C
!1121144703-014+00003310811+01736+003-03+1025-033+110831245+01650023176C
!1121144703-014+00003310811+01736+003-03+1035-033+110831245+01650023176C
!1121144703-014+00003310811+01736+003-03+1055-033+110831245+01650023176C
!1121144703-014+00003310811+01736+003-03+1073-033+110831245+01650023176C
*/

#include <TimerOne.h>         // timer lib functions for using DUE timers and callbacks.
#include <Tone.h>
#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>            // so we can provide easily modifiable config file
#include <LedControl.h>       // so can display airspeed and Dynon AOA %

// #include <Gaussian.h>         // gaussian lib used for avg out AOA values.
// #include <LinkedList.h>       // linked list is also required.
// #include <GaussianAverage.h>  // more info at https://github.com/ivanseidel/Gaussian

// Output serial debug infomation. (comment out the following line to turn off serial debug output)
#define SHOW_SERIAL_DEBUG 
// #define SHOW_SERIAL_DEBUG_PLUS

// Average out AOA values to give a smoother transtition between values.
//#define USE_AOA_AVERAGE
// set how many previous AOA values we store to create avg and help smooth out the data values.
#define AOA_HISTORY_MAX       5

// set Freq of callback function used to create the pulse tones.
#define FREQ_OF_FUNCTION      100       // times per second

// set the sample rate of how often to pull the AOA value from the serial stream.
// example 
// 1 = sample every AOA value received from the efis.
// 5 = skip every 5 lines then sample the AOA value.
#define SERIAL_SAMPLE_RATE    3  // note, this means test data needs to be sent N times too

// Min airspeed in knots before the audio tone is turned on
// This is useful because if your on the ground you don't need a beeping in your ear.
#define MUTE_AUDIO_UNDER_IAS  15

// Expected serial string lengths
#define DYNON_SERIAL_LEN              53  // with CR LF counted
#define DYNON_SKYVIEW_SERIAL_LEN      74
#define GARMIN_G3X_SERIAL_LEN         59  

// array postions for aoa values
#define POS_HIGH_TONE_AOA_STALL   0      // % (and above) where stall happens.
#define POS_HIGH_TONE_AOA_START   1      // % (and above) where high tone starts
#define POS_LOW_TONE_AOA_SOLID    2      // % (and above) where a solid low tone is played.
#define POS_LOW_TONE_AOA_START    3      // % (and above) where low 
#define ALL      0                       // array index, now all units share one slot

int AoaToneMatrix[1][5];          // tone matrix for different efis units.
// unsigned char WhichEFIS = D100;   // detect which efis is being read and set store it here.

// general tone settings. Tone Pulse Per Sec (PPS)  (used for all efis units)
#define HIGH_TONE_STALL_PPS       20      // how many PPS to play during stall
#define HIGH_TONE_PPS_MAX         6.5     // 6.5   
#define HIGH_TONE_PPS_MIN         1.5     // 1.5
#define HIGH_TONE_HZ              1600    // freq of high tone
//#define HIGH_TONE2_HZ             1500    // a 2nd high tone that it will cycle between (if defined)
#define LOW_TONE_PPS_MAX          8.5
#define LOW_TONE_PPS_MIN          1.5
#define LOW_TONE_HZ               400     // freq of low tone

#define BAUDRATE_EFIS         115200  // baud rate for D100/D10 and dynon and G3x

#define TONE_PIN              10    // audio output
#define PIN_LED1              13    // internal LED for showing AOA status.
#define PIN_LED2              54    // aka A0. external LED for showing serial input.
#define DEBUG_PIN             41    // if jumpered low, we read input from the USB port
#define SEVENSEG_DATA_PIN      4    // 7 segment display
#define SEVENSEG_CLK_PIN       6    //  "
#define SEVENSEG_CS_PIN        5    //  "

#define PULSE_TONE            1
#define SOLID_TONE            2
#define TONE_OFF              3
#define STARTUP_TONES_DELAY   120
#define STARTUP_TONES_DELAY2  300


volatile uint8_t toneState = false;
volatile uint8_t toneMode = TONE_OFF;    // current mode of tone.  PULSE_TONE, SOLID_TONE, or TONE_OFF
volatile boolean highTone = false;       // are we playing high tone or low tone?
volatile uint16_t toneFreq = 0;       // store current freq of tone playing.
volatile uint8_t AOA = 0;             // avaraged AOA value is stored here.

uint8_t sampleCount = 0;              // used for pulling 1/N sample message from input
float pps = 0;                        // store current PPS of tone (used for debuging) 
uint8_t liveAOA;                      // realtime AOA value.
uint8_t lastAOA = 111;                // save last AOA value here. impossible value so start up works 
unsigned int ALT = 0;                 // hold ALT (only used for debuging)
int ASI = 0;                          // live Air Speed Indicated
unsigned long cyclesWOSerialData = 0; // keep track if not serial data is recieved.
//unsigned char efisTypeDetected = 0; // auto detect which efis is being used. 0 no detection. 1 = D series, 2 = Skyview
//GaussianAverage myAverageAOA = GaussianAverage(AOA_HISTORY_MAX);

float ppsLookup[100];                 // pre-calculate pps values for all 100 possible AOA --- performance

boolean usbMode;                      // Rcv data via USB for bench testing

// SD_CS_PIN should be defined according to your hardware setup
const uint8_t SD_CS_PIN = 53; 
SdFat sd;                             // SD card stuff...
File myFile;

// vars for converting AOA scale value to PPS scale value.
int OldRange,  OldValue;
float NewRange, NewValue;

char inChar;                    // store single serial input char here.
#define MAXSIZE 90              // max length of string
char input[MAXSIZE+1];          // buffer for full serial data input string.
uint8_t inputPos = 0;           // current postion of serial data.
char tempBuf[90];               // misc char buffer used for debug


Tone tonePlayer[1];    // create
LedControl myDisp = LedControl(SEVENSEG_DATA_PIN, SEVENSEG_CLK_PIN, SEVENSEG_CS_PIN, 1);  // Data, Clk, CS, #Devices

bool sdStart(){
  Serial.print("Initializing SD card...");  
  if (!sd.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    sdFailed();
    return false;
  }
  Serial.println("initialization done.");
  
  myFile = sd.open("aoaconfig.txt", FILE_READ);
  if (!myFile) {
    Serial.println("File Open of aoaconfig.txt failed");
    sdFailed();
    return false;
  }

  if (myFile) {
    Serial.println("aoaconfig.txt:");

    readConfig();  // pick up configuration values from SD
    
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening aoaconfig.txt");
    sdFailed();
    return false;
  }  
  return true;
}

float preCalcPPS(int AOA) {               // overrides global AOA
  // used by setup to pre-load AOA array
  
  // check AOA value and set tone and pauses between tones according to 
  if(AOA >= AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_STALL]) {
    return (HIGH_TONE_STALL_PPS);
    
    // in this code "Old" refers to the inputs before rescaling, that is AOA values
    // "New" refers to output of the math, which are PPS values
    
  } else if(AOA >= AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START]) {
    // play HIGH tone at Pulse Rate 1.5 PPS to 6.2 PPS (depending on AOA value)

    OldValue = AOA-(AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START]-1);
    // scale number using this. http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
    OldRange = AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_STALL] - AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START]; //20 - 1;  //(OldMax - OldMin)  
    NewRange = HIGH_TONE_PPS_MAX - HIGH_TONE_PPS_MIN; // (NewMax - NewMin)  
    NewValue = (((OldValue - 1) * NewRange) / OldRange) + HIGH_TONE_PPS_MIN; //(((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    return (NewValue);
  } else if(AOA >= AoaToneMatrix[ALL][POS_LOW_TONE_AOA_SOLID]) {
    // play a steady LOW tone
    return 0.0;
  } else if(AOA >= AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START]) {
    // play LOW tone at Pulse Rate 1.5 PPS to 8.2 PPS (depending on AOA value)
    // scale number using this. http://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio
    OldValue = AOA - AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START];
    OldRange = AoaToneMatrix[ALL][POS_LOW_TONE_AOA_SOLID] - AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START]; //40 - 1;  //(OldMax - OldMin)  
    NewRange = LOW_TONE_PPS_MAX - LOW_TONE_PPS_MIN; // (NewMax - NewMin)  
    NewValue = (((OldValue - 1) * NewRange) / OldRange) + LOW_TONE_PPS_MIN; //(((OldValue - OldMin) * NewRange) / OldRange) + NewMin
    return (NewValue);
  } else {  
    // low AOA, below any tones
    return 0.0;
  }  
}

void setup() {    // put your setup code here, to run once:
  
  pinMode(TONE_PIN, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(DEBUG_PIN, INPUT_PULLUP);   // jumper low to enable input from USB port
  tonePlayer[0].begin(TONE_PIN);      // assign onto pin 11
  setFrequencytone(400);
  
  usbMode = !digitalRead(DEBUG_PIN);  // if jumpered to ground we will read input from PC via USB

  myDisp.shutdown(0,false); /* Normal functioning of device device with address 0 */
  myDisp.setIntensity(0,8); /* Define medium(8) intensity of display device with address 0 */
  myDisp.clearDisplay(0);   /* Clear display device with address 0 */ 
  
  myDisp.setChar(0,6,'A',false);
  myDisp.setChar(0,5,'o',false);
  myDisp.setChar(0,4,'A',false);

  myDisp.setChar(0, 3, VERSION[0], false);
  myDisp.setChar(0, 2, VERSION[1], false);
  myDisp.setChar(0, 1, VERSION[2], false);

  delay(5000);
  myDisp.clearDisplay(0);   /* Clear display device with address 0 */
  
  
  Serial.begin(115200);   //Init hardware serial port (ouput to computer for debug)
  Serial.println("FlyOnSpeed version 1.7.1 AOA Tone Gen");
  if (usbMode) {Serial.println("Input being read from PC/USB port");}
  else {Serial.println("Input being read from Serial3 port");}
  Serial3.begin(BAUDRATE_EFIS);  //Init hardware serial port (input from EFIS)
  

  // startup the SD card
  if (sdStart == true) {
    Serial.println("SD opened and read");
  }
  else {
    Serial.println("SD did not open, use default values");
  }
  
  // startup tone sequence.
  digitalWrite(PIN_LED2, 1);
  setFrequencytone(400);
  delay(STARTUP_TONES_DELAY2);
  setFrequencytone(600);
  delay(STARTUP_TONES_DELAY2);
  setFrequencytone(800);
  delay(STARTUP_TONES_DELAY2);
  setFrequencytone(0);
  digitalWrite(PIN_LED2, 0);

  myDisp.setChar(0,1,'-',false);  // indicate no data yet
  myDisp.setChar(0,0,'-',false);
  
  // timer callback is used for turning tone on/off.
  Timer1.initialize(500000);                 // microseconds
  Timer1.attachInterrupt(tonePlayHandler);
  // Timer1.setFrequency(FREQ_OF_FUNCTION);  // how often to run call back function
  Timer1.setPeriod(1000000 / FREQ_OF_FUNCTION);  // for TimerOne library
  Timer1.start();

  // Dynon Skyview settings.
  // For skyview export ahrs data out one of the 5 serialports (only needs serial output to AOA box, no serial input needed)

  // Original FlyOnSpeed v1.6 SKYVIEW set points, for reference
  /*
  AoaToneMatrix[SKYVIEW][POS_HIGH_TONE_AOA_STALL] = 80;      // % (and above) where stall happens.
  AoaToneMatrix[SKYVIEW][POS_HIGH_TONE_AOA_START] = 66;      // % (and above) where high tone starts
  AoaToneMatrix[SKYVIEW][POS_LOW_TONE_AOA_SOLID]  = 55;      // % (and above) where a solid low tone is played.
  AoaToneMatrix[SKYVIEW][POS_LOW_TONE_AOA_START]  = 40;      // % (and above) where low 
  */
  // Dynon D100 and D10 settings.  For N980RV
  // updated 8/12/2026
  AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_STALL] =  47;      // % (and above) where stall happens.
  AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START] =  38;      // % (and above) where high tone starts
  AoaToneMatrix[ALL][POS_LOW_TONE_AOA_SOLID]  =  28;      // % (and above) where a solid low tone is played.
  AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START]  =   7;      // % (and above) where low  *****  BGL ****

  for (int i=0; i<100; i++) {  // loop 100 times to pre calc pps vs AOA
    ppsLookup[i] =  preCalcPPS(i);
    Serial.print("AOA: ");
    Serial.print(i);
    Serial.print(" PPS: ");
    Serial.println(ppsLookup[i]);
  }  

  input[MAXSIZE] = '\0';           // put termination on string

}  // *************** END SETUP *****************

void sdFailed() {
  // characters are right to left

  myDisp.setChar(0,7,'n',false);
  myDisp.setChar(0,6,'o',false);
  myDisp.setChar(0,4,'5',false);
  myDisp.setChar(0,3,'d',false);
  myDisp.setChar(0,1,'c',false);
  myDisp.setChar(0,0,'d',false);
  while(true);
}

void readConfig(){                  // read the SD card aoaconfig.txt
  if (myFile.available()) {
    String list = myFile.readStringUntil('\n');
    Serial.print("Line 1: ");
    Serial.println(list);           // record 1

    list = myFile.readStringUntil('\n');  // value 1
    list.trim();  
    AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START] = list.toInt();
    Serial.println(AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START]);    

    list = myFile.readStringUntil('\n');
    list.trim();
    AoaToneMatrix[ALL][POS_LOW_TONE_AOA_SOLID] = list.toInt();
    Serial.println(AoaToneMatrix[ALL][POS_LOW_TONE_AOA_SOLID]); 
    
    list = myFile.readStringUntil('\n');
    list.trim();
    AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START] = list.toInt();
    Serial.println(AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START]); 
    
    list = myFile.readStringUntil('\n');
    list.trim();
    AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_STALL] = list.toInt();
    Serial.println(AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_STALL]); 

    myFile.close();
  }
}

// We use our own counter for how often we should pause between tones (pulses per sec PPS)
int cycleCounter = 0;
volatile int cycleCounterResetAt = FREQ_OF_FUNCTION;
uint8_t Tone2FlipFlop = false;

void tonePlayHandler(){         // call back function for interrupt
  cycleCounter++;
  // check if our counter has reach the reset mark.  if so flip the tone on/off and start again.
  // cycleCounterResetAt is set by the PPS set function.
  if(cycleCounter >= cycleCounterResetAt) {
    toneState = toneState ^ 1;  // flip tone state
    cycleCounter = 0;
  } else {
    return;
  }
  
  if(toneMode==TONE_OFF) {
    setFrequencytone(0);  // if tone off skip the rest.
    return;
  }
  if(toneMode==SOLID_TONE) {  // check for a solid tone.
    setFrequencytone(LOW_TONE_HZ);
    return; // skip the rest
  }

  // cylce tone on/off depending on toneState which is flipped in code above.
  if(toneState) {
     digitalWrite(PIN_LED1, digitalRead(PIN_LED1)^1);  // cycle led on/off
     //sprintf(tempBuf, "handler() AOA:%i ASI:%i tone: %i PPS:%f handlerFreq: %i",AOA,ASI,toneFreq,pps, handlerFreq);
     //Serial.println(tempBuf);
     if(highTone) {
// check if we want 2 different tones for the high tone mode.      
#ifdef HIGH_TONE2_HZ
        Tone2FlipFlop = Tone2FlipFlop ^ 1; 
        if(Tone2FlipFlop)
          setFrequencytone(HIGH_TONE_HZ);
        else
          setFrequencytone(HIGH_TONE2_HZ);
#else
        setFrequencytone(HIGH_TONE_HZ);
#endif
     } else {
        setFrequencytone(LOW_TONE_HZ);
     }

  } else {
    setFrequencytone(0);
  }
}

void displayAOA() {   // show rcvd AOA number on LCD
  myDisp.setDigit(0,1,(byte)(AOA / 10),false);
  myDisp.setDigit(0,0,(byte)(AOA % 10),false);

/*
  int tempAS = ASI;
  for (int i = 2; i >= 0; i--) {
    myDisp.setDigit(0, i+4, (byte)(tempAS%10), false);
    tempAS /= 10;
  }
*/
}

void checkAOA() {
  if(ASI <= MUTE_AUDIO_UNDER_IAS) {
#ifdef SHOW_SERIAL_DEBUG    
  // show audio muted and debug info.
    sprintf(tempBuf, "AUDIO MUTED: Airspeed too low. Min:%i ASI:%i",MUTE_AUDIO_UNDER_IAS, ASI);
    Serial.println(tempBuf);
#endif
    toneMode = TONE_OFF;
    displayAOA();
    return;
  }

  if(lastAOA == AOA) {
    return;         // do nothing if the AOA value has not changed.
  }

  displayAOA();     // on LCD
  float ppstemp = ppsLookup[AOA];   // quick lookup pps for this AOA!
  
  // check AOA value and set tone and pauses between tones according to 
  if (AOA >= AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_STALL]) {
    // play 20 pps HIGH tone
    highTone = true;
    Serial.println("highTone");
    setPPSTone(ppstemp);
    toneMode = PULSE_TONE;
    
  } else if (AOA >= AoaToneMatrix[ALL][POS_HIGH_TONE_AOA_START]) {
    highTone = true;
    toneMode = PULSE_TONE;
    setPPSTone(ppstemp);
    
  } else if (AOA >= AoaToneMatrix[ALL][POS_LOW_TONE_AOA_SOLID]) {
    // play a steady LOW tone
    highTone = false;
    toneMode = SOLID_TONE;
    
  } else if (AOA >= AoaToneMatrix[ALL][POS_LOW_TONE_AOA_START]) {
    toneMode = PULSE_TONE;
    highTone = false;
    setPPSTone(ppstemp);
    
  } else {
    toneMode = TONE_OFF;
    setPPSTone(1.0);          // avoid fast ticking audio noise
    Serial.println("in TONE_OFF range");
  }

  lastAOA = AOA;
#ifdef SHOW_SERIAL_DEBUG    
  // show serial debug info.
  sprintf(tempBuf, "TONE AOA:%i Live:%i ASI:%ikts ALT:%i PPS:%f cycleCounterResetAt: %i",AOA, liveAOA, ASI, ALT, ppstemp, cycleCounterResetAt);
  Serial.println(tempBuf);
#endif
}

void setPPSTone(float newPPS) {
  // set PPS by setting cycleCounterResetAt which is used in the tonePlayHandler()
  // Note, I don't understand the *1.5, tone should switch to on, then off at 2x PPS
  // I would think.  But this is the code from the original FlyOnSpeed
  // cycleCounterResetAt = (1/(newPPS*1.5)) * FREQ_OF_FUNCTION;
  cycleCounterResetAt = FREQ_OF_FUNCTION / (newPPS*1.5);  // refactor to eliminate 1 float operation
  pps = newPPS;  // store pps for debug purposes.
}

int inputAvailable() {   // use like Serial.available() to handle two inputs
  if (usbMode) {           // debug, use PC USB serial port
    return Serial.available();
  }
  else {
    return Serial3.available();   // otherwise use Serial3  
  }
}

char readChar() {
  if (usbMode) {
    return (char)Serial.read();  // read in char from PC/Debug/USB
  }
  else {
    return (char)Serial3.read(); // read in char from Dynon
  }     
}

// counter for how often to cycle led and sample AOA from serial data.
int LedCountDown = SERIAL_SAMPLE_RATE; 

// ====================================================================
// main loop of app 
// ====================================================================
void loop() {
      // check for serial in from efis.      

// sampleCount, if not zero, pull and discard data.  on \n decrement sampleCount         
      while(inputAvailable() && (sampleCount > 0)) {
        inChar = readChar();
        if (inChar == '\n') { 
          /*
          Serial.print("skipped line @ sampleCount ");          
          Serial.println(sampleCount);
          */
          sampleCount -= 1;        // EOL, decrement
          inputPos = 0; 
        }  
      }      

      // data we want to look at
      while(inputAvailable() && (sampleCount == 0)) {     // from whichever port
        inChar = readChar();       

        if (inputPos >= MAXSIZE) {     // first thing, check for overrun
          inputPos = 0;                // reset pointer
          Serial.println("OVER FLOW DATA");
          input[MAXSIZE] = '\0';       // null terminate it
          Serial.println(input);
        }

        input[inputPos]=inChar;         // save into string buffer.
        inputPos++;                     // increment the string buffer postion.
        cyclesWOSerialData = 0;         // reset the var thats keep track of when we last had serial data
                                        // (used for turning off tone if no serial data found)
        if (inputPos == 1) {
          myDisp.setChar(0,7,'d',false);  // indicate Rx of data
        }

        // check for dynon skyview data. based on length of string and if the 1st 2 chars match
        if (inChar == '\n' && inputPos == DYNON_SKYVIEW_SERIAL_LEN && input[0]=='!' && input[1]=='1') {
          // WhichEFIS = SKYVIEW;  // set skyview efis detected.
          // skyview data starts with a '!1'... the D series efis has no line prefix.  
          tempBuf[0] = input[43]; // get the 2 bytes for aoa on skyview.
          tempBuf[1] = input[44]; // 
          tempBuf[2] = '\0'; // term string  (needed for when we convert a string to a int in the next line)
          // Skyview may put "XX" into AOA when it is not available
          if (input[42] != 'X') { 
            liveAOA = strtol(tempBuf, NULL, 10);  //convert to int

            // get AOA value (check if we are using a Gaussian function to smooth the values)
#ifdef USE_AOA_AVERAGE
            myAverageAOA += liveAOA;            // store and calcute the mean average AOA from previous values.
            AOA = myAverageAOA.process().mean;
#else          
            AOA = liveAOA;  // else just use the live AOA value.
#endif
          }  // end handling of "XX" AOA
          // get ASI (4 digits) but we only want the first 3. we don't need a 10/th of a knot
          tempBuf[0] = input[23]; //
          tempBuf[1] = input[24]; //
          tempBuf[2] = input[25]; //
          tempBuf[3] = '\0'; //
          ASI = strtol(tempBuf, NULL, 10); //convert to int (data comes in knots already on skyview)

#ifdef SHOW_SERIAL_DEBUG_PLUS    
  // show serial debug info for skyview data found.
  // can't edit the tempBuf here.. was causing crash. Christopher. 9/26/2018
  //sprintf(tempBuf, "SKYVIEW AOA:%i Live:%i ASI:%ikts ALT:%i PPS:%f cycleCounterResetAt: %i",AOA, liveAOA, ASI, ALT, pps, cycleCounterResetAt);
  //Serial.println(tempBuf);
          Serial.print("SKYVIEW AOA ");
          if (input[42] != 'X') { Serial.println(AOA);
          } else { Serial.println("XX");}
#endif
          if (input[42] != 'X') {
            validAOADataFound();  // run function to process tone.  because we found a valid AOA value.
          }

// -------------------  END OF SKYVIEW LOGIC ----------------------------------------
// -------------------  START D10A LOGIC --------------------------------------------
          
        } else if (inChar == '\n' && inputPos == 53) {  // is EOL? 
          // WhichEFIS = D100;  // set D100 or D10A efis detected
          // get AOA from dynon efis string.  details of format can be found from dynon pdf manual.
          tempBuf[0] = input[39]; //
          tempBuf[1] = input[40]; //
          tempBuf[2] = '\0'; //term string (needed for converting a string to a int in the next line)
          liveAOA = strtol(tempBuf, NULL, 10); //convert to int

          // get AOA value (check if we are using a Gaussian function to smooth the values)
#ifdef USE_AOA_AVERAGE
          myAverageAOA += liveAOA;            // store and calcute the mean average AOA from previous values.
          AOA = myAverageAOA.process().mean;
#else          
          AOA = liveAOA;  // else just use the live AOA value.
#endif
      
          // get ASI (4 digits) but we only want the first 3 because its in meters per second.
          tempBuf[0] = input[20]; //
          tempBuf[1] = input[21]; //
          tempBuf[2] = input[22]; //
          tempBuf[3] = '\0'; // terminate string (needed for converting a string to a int in the next line)
          ASI = strtol(tempBuf, NULL, 10) * 1.943; //convert to int (and from m/s to knots)

          // get ALT (4 digits) in meters.. this is really not used for this application.  we just collect it to show in debug.
          tempBuf[0] = input[25]; //
          tempBuf[1] = input[26]; //
          tempBuf[2] = input[27]; //
          tempBuf[3] = input[28]; //
          tempBuf[4] = '\0'; //
          // ALT = (strtol(tempBuf, NULL, 10) * 0.328) * 10; //convert to long (and from meters to feet)
          validAOADataFound();    // run function to process tone.  because we found a valid AOA value.

#ifdef SHOW_SERIAL_DEBUG    
          // Serial.print("D100 AOA ");
          // Serial.println(AOA);
#endif
        // ----------------- END OF D10A/D100 LOGIC ----------------------------
        // ----------------- START G3X LOGIC -----------------------------------
        } else if(inChar == '\n' && inputPos == 59) { 
          // WhichEFIS = G3X;  // set efis detected
          // efisTypeDetected != SKYVIEW has been added to make sure we don't confuse skyview data for d10 data and try to process..
          // get AOA from dynon efis string.  details of format can be found from dynon pdf manual.
          tempBuf[0] = input[43]; //
          tempBuf[1] = input[44]; //
          tempBuf[2] = '\0'; //term string (needed for converting a string to a int in the next line)
          liveAOA = strtol(tempBuf, NULL, 10); //convert to int

          // get AOA value (check if we are using a Gaussian function to smooth the values)
#ifdef USE_AOA_AVERAGE
          myAverageAOA += liveAOA;            // store and calcute the mean average AOA from previous values.
          AOA = myAverageAOA.process().mean;
#else          
          AOA = liveAOA;  // else just use the live AOA value.
#endif
      
          // get ASI (4 digits) but we only want the first 3 because 1/10th of a Knot.
          tempBuf[0] = input[23]; //
          tempBuf[1] = input[24]; //
          tempBuf[2] = input[25]; //
          tempBuf[3] = '\0'; // terminate string (needed for converting a string to a int in the next line)
          ASI = strtol(tempBuf, NULL, 10);  //convert to int 

          validAOADataFound();

#ifdef SHOW_SERIAL_DEBUG    
          Serial.print("G3X AOA ");
          Serial.println(AOA);
#endif       
        } 

        if (inChar == '\n') {
          // if we are at the end of line then reset postion to start looking for a new line.
          inputPos = 0; 
          sampleCount = SERIAL_SAMPLE_RATE -1;    // reset
        } 
        
      }  // end while inputAvailable (and is a line we wish to sample)

      // check if no serial has been sent in a while.
      cyclesWOSerialData++;
      // delay(200);                           // *************  MY BENCH DEBUG *************
      if(cyclesWOSerialData > 666000) {
        //TODO:  should we do something more to inidicate to the user there is no serial data coming in?  
        // this might be useful for troubleshooting after a install.
        // perhaps a warning light?
        toneMode = TONE_OFF;        // stop tone
        lastAOA = 0;                // reset AOA
        cyclesWOSerialData == 0;
        digitalWrite(PIN_LED2, 0);  // no EFIS serial data so turn off led.
        myDisp.setChar(0,0,'-',false);  // indicate no data
        myDisp.setChar(0,1,'-',false);
        myDisp.setChar(0,7,' ',false);  // clear the 'd' character
#ifdef SHOW_SERIAL_DEBUG    
       // Serial.println("NO DATA");  // *********** TEMP turn off to avoid overwhelming monitor *********
#endif
      } 
}

// valid AOA data was found so this function will be run.  Here we toggle the leds and see how often we should sample the AOA data.
void validAOADataFound() {
  checkAOA();  // check AOA data from dynon and store result into AOA var.
  LedCountDown --;
  if(LedCountDown<1) {
     digitalWrite(PIN_LED2, digitalRead(PIN_LED2)^1);  // cycle serial RX led on/off
     LedCountDown = SERIAL_SAMPLE_RATE;     
  }
  inputPos = 0;  // reset postion to start of buffer string for next round.
}


void setFrequencytone(uint32_t frequency)
{
  if(frequency < 20 || frequency > 20000) {
    //Serial.print("cancel tone: ");Serial.println(frequency);
    tonePlayer[0].stop();
    return;
  }

  if(toneFreq == frequency) {
    // if the new frequency is the same as the current freq then don't do anything.
    return;
  }
  
  tonePlayer[0].play(frequency);
}


