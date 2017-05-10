#include "FastLED.h"

FASTLED_USING_NAMESPACE

//Led control for blinker lights and a single light strip

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    3
#define CLK_PIN   4
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    14

CRGB leds[NUM_LEDS];
CRGB leds2[NUM_LEDS];

#define BRIGHTNESS          64
#define FRAMES_PER_SECOND  120


int inputLeftBlinker = 8;
int inputRightBlinker = 9;

boolean oikeaVilkku = false;
boolean vasenVilkku = false;

//Global variable from 0-255 used to determine state of lights
unsigned char ledInc = 0;


void setup() {
  pinMode(inputLeftBlinker, INPUT);
  pinMode(inputRightBlinker, INPUT);
  
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<APA102,A0, A1,RGB>(leds2, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

void loop()
{
  //Valonauha punainen edes taas
  fadeToBlackBy( leds2, NUM_LEDS, 20);
  int pos = beatsin16(40,0,NUM_LEDS);
  leds2[pos] += CHSV( 160, 255, 192);
  FastLED.show();

  //Etuvalot valkoiseksi aluksi
  for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::White;
  }
  //Määritä vilkkujen totuusarvot
  if(digitalRead(inputLeftBlinker) == HIGH) {
    vasenVilkku = true;
  } else {
    vasenVilkku = false;
  }

  if(digitalRead(inputRightBlinker == HIGH)) {
    oikeaVilkku = true;
  } else {
    oikeaVilkku = false;
  }

  //Käytä vilkkuja
  if (vasenVilkku == true && oikeaVilkku == false) {
    if(ledInc > 0 && ledInc < (255 / 4)) {
      leds[1] = CRGB::Orange;
    } else if (ledInc > (255 / 4) && ledInc < 2 * (255 / 4) ) {
      leds[1] = CRGB::Orange;
      leds[0] = CRGB::Orange;
    } else if (ledInc > 2 * (255 / 4) && ledInc < 3 * (255 / 4)){
      leds[1] = CRGB::Orange;
      leds[0] = CRGB::Orange;
      leds[4] = CRGB::Orange;
    } else {
      //nothing
    }

  } else if (oikeaVilkku == true && vasenVilkku == false) {
    if(ledInc > 0 && ledInc < (255 / 4)) {
      leds[11] = CRGB::Orange;
    } else if (ledInc > (255 / 4) && ledInc < 2 * (255 / 4) ) {
      leds[11] = CRGB::Orange;
      leds[7] = CRGB::Orange;
    } else if (ledInc > 2 * (255 / 4) && ledInc < 3 * (255 / 4)){
      leds[11] = CRGB::Orange;
      leds[7] = CRGB::Orange;
      leds[8] = CRGB::Orange;
    } else {
      //nothing
    }
    
  } else if (vasenVilkku == true && oikeaVilkku == true) {
    if(ledInc > 0 && ledInc < (255 / 4)) {
      leds[1] = CRGB::Orange;
      leds[11] = CRGB::Orange;
    } else if (ledInc > (255 / 4) && ledInc < 2 * (255 / 4) ) {
      leds[1] = CRGB::Orange;
      leds[0] = CRGB::Orange;
      leds[11] = CRGB::Orange;
      leds[7] = CRGB::Orange;
    } else if (ledInc > 2 * (255 / 4) && ledInc < 3 * (255 / 4)){
      leds[1] = CRGB::Orange;
      leds[0] = CRGB::Orange;
      leds[4] = CRGB::Orange;
      leds[11] = CRGB::Orange;
      leds[7] = CRGB::Orange;
      leds[8] = CRGB::Orange;
    } else {
      //nothing
    }
  } else {
    //nothing
  }

  FastLED.show();
  
  //Add to global variable, amount determines speed of blinkers
  ledInc += 4;
  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND);   
}
