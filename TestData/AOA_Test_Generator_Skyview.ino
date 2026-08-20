// THIS VERSION IS TO SEND DYNON D10A / D100 SERIES DATA STRINGS

// Bryan LaPlante
// Since we are running the AOA tone generator on a Mega, without floating point hardware
// we need to insure that the input buffer does not get overrun.
// When processing every sample, on the Dx Dynon, it appears that overrun did happen on
// the pre 6/17/2026 version.
//
// Dynon can send 60 strings per second.  So we need to confirm that
// even at faster rates there are no overflows, hangs, crashes.
// This is even more critical for the Skyview format which is longer than the Serial
// hardware buffer size
//
// Most critical in the current code is the floating point math that happens when the tone
// is in a range that requires calculation of pps (pulses per second), and that the AOA
// value changed from last time.
//
// At 115,200 Baud, The Dx sentance takes 4.6ms, and would arrive every 16.6ms

#define BAUDRATE_EFIS 115200
#define DELAY_MS   5        // 100mS = 10/sec, 10mS, approx 100/sec
// #define DELAY_MS   100
#define AOA_MIN    10
#define AOA_MAX    50
#define AOA_REPEAT  2       // how many times to send same value

/* ------------ Dynon Skyview TEST DATA, send via Serial Monitor with CR+NL termination
!1121144703-014+00003310811+01736+003-03+1013-033+110831245+01650023176C  -- from Dynon Install Guide
                                           ^^  13 in this case
ADAHRS data starts with !1, others are !2, etc.
Format is 1 based (! is character 1)
AOA is at 44-45 value up to 99, like D100 series
But can be XX for "not available"
!1121144703-014+00003310811+01736+003-03+1010-033+110831245+01650023176C
*/

String sOne = "!1121144703-014+00003310811+01736+003-03+10";  // before two digit AOA
String sTwo = "-033+110831245+01650023176C";                               // after it
 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE_EFIS);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i = AOA_MIN;
  while (true){
    for (int y = 0; y < AOA_REPEAT; y++) {
      Serial.print(sOne);       // first part
      Serial.print(i, DEC);     // the AOA
      Serial.println(sTwo);     // rest of the line
      delay(DELAY_MS);
    }
    i++;
    if (i > AOA_MAX) {i = AOA_MIN;}
  }
}
