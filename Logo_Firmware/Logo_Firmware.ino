// Coding by Mark Donners, The Electronic Engineer
// June 2024
// v1.0
// Spectrum Analyzer
// Youtube Subscriber counter


/* libaries used
FastLED at version 3.6.0                  
GyverMAX7219 at version 1.5               
GyverGFX at version 1.7                   
SPI at version 1.0             
TM1637_RT at version 0.4.0        
ESP32-audioI2S-master at version 2.0.0  
WiFi at version 1.0             
WiFiClientSecure at version 1.0       
SD at version 1.0.5             
FS at version 1.0             
SD_MMC at version 1.0           
SPIFFS at version 1.0           
FFat at version 1.0             
YoutubeApi at version 2.0.0         
ArduinoJson at version 7.0.4        
arduinoFFT at version 1.5.6         
EasyButton at version 2.0.1         
 
 */

#include <FastLED.h>
#include <driver/i2s.h>
#include <GyverMAX7219.h>
#include "TM1637.h"
#include "Audio.h"
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <YoutubeApi.h>
#include <ArduinoJson.h>
#include <arduinoFFT.h>
#include <EasyButton.h>
#include "Settings_and_functions.h"
//variables we don't need to change, but we need them
unsigned long time_last_run = 0;
int16_t samples[SAMPLEBLOCK];
double vReal[SAMPLEBLOCK];
double vImag[SAMPLEBLOCK];
uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;
long SubCountNew = 0;
long SubCountOld = 0;
long SubCountThisRun = 0;
volatile boolean playflag = false;
int16_t sBuffer[bufferLen];

WiFiClientSecure client;
YoutubeApi api(API_KEY, client);
MAX7219 < 4, 2, 15, 2, 4 > mtrx; // 4 horizon, 2 vert, cs,din, clk
TM1637 TM;
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
CRGB leds[NUM_LEDS]; // Define the array of leds
Audio audio;              //Definition of our audio board instance
EasyButton button1(26, 40, true, true);
TaskHandle_t LedTask; // instance of a task we need when operating two processor cores

void onPressed() {                                                    //**
  Serial.println(" Button has been pressed!");                    //**
  playflag = true;
  delay(50);

}

void setup() {
  button1.begin();
  button1.onPressed(onPressed);
  Serial.begin(115200);
  delay(500);

  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);
  Serial.println("I2S Mic iput installed on Port 1");

  //setup display
  TM.begin(18, 21);       //  clock pin, data pin
  TM.displayClear();
  TM.setBrightness(7);
  TM.displayInt(123456);
  Serial.println("Display installed");

  // setup dot matrix
  mtrx.begin();       // запускаем
  mtrx.setBright(1);  // яркость 0..15
  mtrx.setConnection(GM_RIGHT_BOTTOM_LEFT);
  mtrx.clear();
  mtrx.println(" You ");
  mtrx.println("Tube");
  mtrx.update();
  delay(100);
  Serial.println("Dot Matrix installed");

  //wifi
  Setup_Wifi();
  client.setInsecure();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  //
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "LedTask",     /* name of task. */
    20000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    4,           /* priority of the task */
    &LedTask,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  //Serial.println("resetting");
  FastLED.addLeds<WS2812, LedPin, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(10);



  Serial.println("Fastled Ok now audio");
  //audio initialization
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(18); // 0...21
  bool spiffsOK = SPIFFS.begin();

  Serial.printf("SPIFFS Status: %d\n", spiffsOK);
  //audio.connecttoFS(SPIFFS, "wheel.mp3");
  Serial.println("mp3 opened");
  delay(1000);
  randomSeed(analogRead(0));
  Serial.println("Setup is done....");

  demoIntro();
  delay(1000);
}




void loop() {

  // read potmeter for volume
  int VolOut=map(analogRead(PotmeterVolumePin), 0, 4095, 1, 21);
  audio.setVolume(VolOut); // 0...21
  //Serial.println(VolOut);
  size_t bytesRead = 0;
  int TempADC = 0;
  // during the whole time, using core 1 to check input switch and feed audio player
  button1.read();
  int rangelimit = 3000;

  // Get I2S data and place in data buffer
  size_t bytesIn = 0;

  //############ Step 1: read samples from the I2S Buffer ##################
  i2s_read(I2S_PORT,
           (void*)samples,
           sizeof(samples),
           &bytesRead,   // workaround This is the actual buffer size last half will be empty but why?
           portMAX_DELAY); // no timeout

  if (bytesRead != sizeof(samples)) {
    Serial.printf("Could only read %u bytes of %u in FillBufferI2S()\n", bytesRead, sizeof(samples));
  }

  //############ Step 2: compensate for Channel number and offset, safe all to vReal Array   ############
  for (uint16_t i = 0; i < ARRAYSIZE(samples); i++) {
    vReal[i] = offset - samples[i];
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }

  //############ Step 3: Do FFT on the VReal array  ############
  // compute FFT
  FFT.DCRemoval();
  FFT.Windowing(vReal, SAMPLEBLOCK, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLEBLOCK, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLEBLOCK);
  FFT.MajorPeak(vReal, SAMPLEBLOCK, samplingFrequency);
  for (int i = 0; i < numBands; i++) {
    FreqBins[i] = 0;
  }

  //############ Step 4: Fill the frequency bins with the FFT Samples ############
  for (int i = 2; i < SAMPLEBLOCK / 2; i++) {
    if (vReal[i] > NoiseTresshold) {
      int freq = BucketFrequency(i);
      int iBand = 0;
      while (iBand < numBands) {
        if (freq < BandCutoffTable[iBand])break;
        iBand++;
      }
      if (iBand > numBands)iBand = numBands;
      FreqBins[iBand] += vReal[i];
    }
  }

  //############ Step 5: Averaging and making it all fit on screen
  static float lastAllBandsPeak = 0.0f;
  float allBandsPeak = 0;
  for (int i = 0; i < numBands; i++) {
    if (FreqBins[i] > allBandsPeak) {
      allBandsPeak = FreqBins[i];
    }
  }
  // now max peak has been determined
  if (allBandsPeak < 1)allBandsPeak = 1;
  allBandsPeak = max(allBandsPeak, ((lastAllBandsPeak * (GAIN_DAMPEN - 1)) + allBandsPeak) / GAIN_DAMPEN); // Dampen rate of change a little bit on way down
  lastAllBandsPeak = allBandsPeak;
  if (allBandsPeak < 80000)allBandsPeak = 80000;
  for (int i = 0; i < numBands; i++)FreqBins[i] /= (allBandsPeak * 1.0f);

  // Process the FFT data into bar heights
  for (int band = 0; band < numBands; band++) {
    int barHeight = FreqBins[band] * MatrixHeight; //(AMPLITUDE);

    // Small amount of averaging between frames
    barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;

    // Save oldBarHeights for averaging later
    oldBarHeights[band] = barHeight;

  }

  // now everything is to scale for matrix.
  //############ Step 5: put it on screen
  mtrx.clear();
  for (int moveX = 0; moveX < numBands; moveX++) {
    int SetY = oldBarHeights[moveX];
    // now write to screen
    mtrx.fastLineV(moveX, 15 - SetY, 15, 1); // int x, int y0, int y1,1   horizontal line, fill - GFX_CLEAR/GFX_FILL/GFX_STROKE
  }
  mtrx.update();

}//main


void audio_eof_mp3(const char *info) { //end of file
  Serial.print("eof_mp3 "); Serial.println(info);
}



// ** code for second Uc core
void Task1code( void * pvParameters ) {
  esp_task_wdt_init(30, false);
  delay(1000);
  Serial.print("Led task is running on core:  ");
  Serial.println(xPortGetCoreID());
  for (;;) {
    delay(1); // wdt
    audio.loop();

    if (playflag == true) {  // let us start a new round
      playflag = false;
      audio.connecttoFS(SPIFFS, "intro.mp3");
      playflag = false;
    }


    // check stats every minute
    // stats changed? play sound
    //audio.connecttoFS(SPIFFS, "intro.mp3");
    if (millis() - time_last_run > Stat_Pull_interval_ms) {
      time_last_run = millis();
      if (!audio.isRunning()) {
        if (api.getChannelStatistics(CHANNEL_ID)) {
          SubCountNew = api.channelStats.subscriberCount;
         // SubCountNew=SubCountOld+1; only for simulation
          Serial.printf("Number of subscribers: %d\n",SubCountNew);
          TM.displayInt(SubCountNew);
          SubCountThisRun+=(SubCountNew-SubCountOld);
          
          if (SubCountThisRun>9){
            audio.connecttoFS(SPIFFS, "10newsubs.mp3");
            SubCountThisRun=0;
            SubCountOld=SubCountNew;
          }
          if (SubCountNew<SubCountOld){
            audio.connecttoFS(SPIFFS, "lose.mp3");
          }else if ((SubCountNew-SubCountOld)>0){
                 audio.connecttoFS(SPIFFS, "newsub.mp3");
                }
          SubCountOld=SubCountNew;
        }
      }
    }

   rainbow_wave(10,10);  
  } //for section of second core code
} //*** end of code for second Uc core

void demoIntro() {
  audio.connecttoFS(SPIFFS, "intro.mp3");

}




/********************************************************************************************************************************
 * ** sub function to make rainbowcolors on ledstrip                                                                                                   **
 ********************************************************************************************************************************/
void rainbow_wave(uint8_t thisSpeed, uint8_t deltaHue) {      // The fill_rainbow call doesn't support brightness levels.

  // uint8_t thisHue = beatsin8(thisSpeed,0,255);                 // A simple rainbow wave.
  uint8_t thisHue = beat8(thisSpeed, 255); // A simple rainbow march.
  fill_rainbow(leds, NUM_LEDS, thisHue, deltaHue);   // Use FastLED's fill_rainbow routine.
  FastLED.show();
} // rainbow_wave()
