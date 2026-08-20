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
String sOne = "xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx20xxxxxxxxxx";
String sTwo = "xxxxxxxxxxxxxxxxxxx031xxxxxxxxxxxxxxxxx24xxxxxxxxxx";
 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE_EFIS);

}

void loop() {
  // put your main code here, to run repeatedly:
  while (true){
    Serial.println(sOne);
    delay(15);
    Serial.println(sTwo);
    delay(15);
  }
}
