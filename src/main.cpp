#include <Arduino.h>
#include <Ticker.h>
#include <DNSServer.h>
#include <ESPUI.h>
#include <WiFi.h>
#include <config.h>
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// WIFI CONSTANTS
const char *ssid = WIFI_AP_SSID;
const char *password = WIFI_AP_PASSWORD;

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;

// PINS DEFINITION
#define LED_PIN 18
#define DHT_PIN 5
#define MIC_PIN 36

#define NUM_LEDS 4
#define COLOR_ORDER GRB

// SYSTEM STATE CONSTANTS
#define BOOTING "BOOTING"
#define COLOR_CONTROL_MODE "COLOR_CONTROL_MODE"
#define RAINBOW_MODE "Rainbow"
#define OCEAN_MODE "Ocean"
#define CLOUD_MODE "Cloud"
#define LAVA_MODE "Lava"
#define FOREST_MODE "Forest"
#define TEMPERATURE_MODE "TEMPERATURE_MODE"
#define MUSIC_MODE "MUSIC MODE"
#define OFF "OFF"
#define SHUTDOWN "SHUTDOWN"
const char *LAMP_MODE = BOOTING;

// DHT SENSOR CONSTANT
#define DHTTYPE    DHT11

// TOUCH SYSTEM CONSTANT
#define TOUCH_THRESHOLD 60

// Functions declarations
void handleUnrecognized(const char *command);
void writeInternalTemperature();
void showColor(int R, int G, int B);
void processBrightness();
void turnOnLED();
void turnOffLED();
void switchSystem(Control sender, int type);
void switchLED(Control sender, int type);
void switchRainbow(Control sender, int type);
void switchTemperature(Control sender, int type);
void sliderBrightness(Control sender, int type);
void sliderRGBR(Control sender, int type);
void sliderRGBG(Control sender, int type);
void sliderRGBB(Control sender, int type);
void sliderRainbowSpeed(Control sender, int type);
void colorPalettePad(Control sender, int type);
void runRainbow();
void runTemperature();
void runMusic();
void measureTemperatureAndHumidity();
void updateWebData();
void touchCallback();
void FillLEDsFromPaletteColors(uint8_t);
void ChangePalettePeriodically();
void SetupTotallyRandomPalette();
void SetupPurpleAndGreenPalette();
void SetupBlackAndWhiteStripedPalette();
void FillLEDsFromPaletteColors( uint8_t colorIndex);
void enterDeepSleep();
// Variables declarations
int UPDATES_PER_SECOND = 20; // use for the animations
int temperatureGradientMaxLimit = 30;
int temperatureGradientMinLimit = 0;

// Sensors/LEDs values
int R = 125;
int G = 125;
int B = 125;
int brightness = 240;
uint32_t pixel1Color = R/2.55;
uint32_t pixel2Color = G/2.55;
uint32_t pixel3Color = B/2.55;
float temperature = 20; // placehold
int humidity = 50; // placeholder
double reading;

// Music analysis variables
CRGB::HTMLColorCode equalizerColorSchemes[17][4] = {
  {CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black}, {CRGB::Green, CRGB::Black, CRGB::Black, CRGB::Black},
  {CRGB::Green, CRGB::Green,CRGB::Black, CRGB::Black},{CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Black},
  {CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Green},{CRGB::Green,CRGB::Green,CRGB::Green,CRGB::Yellow},
  {CRGB::Green, CRGB::Green, CRGB::Yellow, CRGB::Yellow},{CRGB::Green, CRGB::Yellow, CRGB::Yellow, CRGB::Yellow},
  {CRGB::Yellow, CRGB::Yellow, CRGB::Yellow, CRGB::Yellow},{CRGB::Yellow, CRGB::Yellow, CRGB::Yellow, CRGB::Orange},
  {CRGB::Yellow, CRGB::Yellow, CRGB::Orange, CRGB::Orange},{CRGB::Yellow, CRGB::Orange, CRGB::Orange, CRGB::Orange},
  {CRGB::Orange, CRGB::Orange, CRGB::Orange, CRGB::Orange},{CRGB::Orange, CRGB::Orange, CRGB::Orange, CRGB::Red},
  {CRGB::Orange, CRGB::Orange, CRGB::Red, CRGB::Red},{CRGB::Orange, CRGB::Red, CRGB::Red, CRGB::Red},
  {CRGB::Red, CRGB::Red, CRGB::Red, CRGB::Red}
};
#define SAMPLE_SIZE 100
int samples[SAMPLE_SIZE];
int deltaSamples[SAMPLE_SIZE];
int readingsCounter = 0;

// LED objects/variables
CRGBPalette16 heatPalette = CRGBPalette16(
                               CRGB::WhiteSmoke, CRGB::AliceBlue,
                               CRGB::DeepSkyBlue, CRGB::DeepSkyBlue,
                               CRGB::LightYellow, CRGB::LightYellow,
                               CRGB::Yellow, CRGB::Yellow,
                               CRGB::Orange, CRGB::Orange,
                               CRGB::OrangeRed, CRGB::OrangeRed,
                               CRGB::OrangeRed, CRGB::Red,
                               CRGB::Red, CRGB::Red);
// // LED objects/variables
CRGBPalette16 colorPalettes[] = {
  RainbowColors_p,
  OceanColors_p,
  CloudColors_p,
  LavaColors_p,
  ForestColors_p
};
const char* colorPalettesNames[] = {
    RAINBOW_MODE,
    OCEAN_MODE,
    CLOUD_MODE,
    LAVA_MODE,
    FOREST_MODE
};
int colorPaletteIndex = 0;
const char* colorPaletteName= colorPalettesNames[0];
CRGBPalette16 currentPalette = RainbowColors_p; // for rainbow activity
TBlendType    currentBlending = LINEARBLEND;

CRGB leds[NUM_LEDS]; // define the array of LEDs

// Ticker for periodic activity
Ticker timer1(measureTemperatureAndHumidity, 1000);
Ticker timer2(updateWebData, 100);

// DHT sensor object declaration
DHT_Unified dht(DHT_PIN, DHTTYPE);
uint32_t delayMS;

// Touchpin object declarations
touch_pad_t touchPin;

// Objects declarations

void setup() {
  // Initialize serial
  Serial.begin(115200);
  Serial.println("Booting");

  //Touch initialization
  touchAttachInterrupt(T4, touchCallback, TOUCH_THRESHOLD);
  esp_sleep_enable_touchpad_wakeup();

  // Pins initialization
  pinMode(MIC_PIN, INPUT);

  // Wifi AP setup
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.setHostname(ssid);
  WiFi.softAP(ssid);
  WiFi.softAP(ssid, password);
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // LEDs initialization
  FastLED.addLeds<WS2812B, LED_PIN>(leds, NUM_LEDS);
  // turnOffLED();
  FastLED.show();

  // Web interface definition
  ESPUI.label("Temperature:", COLOR_TURQUOISE,(String) temperature);
  ESPUI.label("Humidity:", COLOR_TURQUOISE,(String) humidity);
  ESPUI.switcher("Status System", true, &switchSystem, COLOR_WETASPHALT);
  ESPUI.switcher("Status LEDs", (LAMP_MODE!=OFF), &switchLED, COLOR_EMERALD);
  ESPUI.slider("Light intensity", &sliderBrightness, COLOR_PETERRIVER,(String) brightness);
  ESPUI.slider("Color RED Value", &sliderRGBR, COLOR_SUNFLOWER,(String) pixel1Color);
  ESPUI.slider("Color GREEN Value", &sliderRGBG, COLOR_SUNFLOWER,(String) pixel2Color);
  ESPUI.slider("Color BLUE Value", &sliderRGBB, COLOR_SUNFLOWER,(String) pixel3Color);
  ESPUI.switcher("Animation Mode", (LAMP_MODE==RAINBOW_MODE), &switchRainbow, COLOR_CARROT);
  ESPUI.label("Color palette:", COLOR_TURQUOISE,(String) colorPaletteName);
  ESPUI.pad("Change color palette", false, &colorPalettePad, COLOR_TURQUOISE);
  ESPUI.slider("Animation speed", &sliderRainbowSpeed, COLOR_CARROT,(String) UPDATES_PER_SECOND);
  ESPUI.switcher("Temperature Mode", (LAMP_MODE==TEMPERATURE_MODE), &switchTemperature, COLOR_CARROT);

  // Server initialization and start
  dnsServer.start(DNS_PORT, "*", apIP);
  //
  // /*
  //  * Optionally you can use HTTP BasicAuth. Keep in mind that this is NOT a
  //  SECURE way of limiting access.
  //  * Anyone who is able to sniff traffic will be able to intercept your password
  //  since it is transmitted in cleartext ESPUI.begin("ESPUI Control", "myuser",
  //  "mypassword");
  // */
  ESPUI.begin("El Platillo volador Control");

  // Sensors initialization
  dht.begin();

  // Timer initialization
  timer1.start();
  timer2.start();

  // Setting default lamp mode
  // LAMP_MODE = MUSIC_MODE;
  LAMP_MODE = RAINBOW_MODE;
  // LAMP_MODE = TEMPERATURE_MODE;

  Serial.println("Booting OK");
}

void loop() {
  if (LAMP_MODE == SHUTDOWN) {
      enterDeepSleep();
  } else {
    // run Server
    dnsServer.processNextRequest();

    if (LAMP_MODE == RAINBOW_MODE) runRainbow();
    if (LAMP_MODE == TEMPERATURE_MODE) runTemperature();
    if (LAMP_MODE == MUSIC_MODE) runMusic();

    // Update timers
    timer1.update();
    timer2.update();
  }
}

void updateWebData() {
  ESPUI.print("Temperature:", (String) temperature);
  ESPUI.print("Humidity:", (String) humidity);
  ESPUI.print("Color palette:", (String) colorPaletteName);
};

void touchCallback() {
  LAMP_MODE = SHUTDOWN;
}

void measureTemperatureAndHumidity() {
  // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      temperature = event.temperature;
      // Serial.print(F("Temperature: "));
      // Serial.print(temperature);
      // Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      humidity = event.relative_humidity;
      // Serial.print(F("Humidity: "));
      // Serial.print(humidity);
      // Serial.println(F("%"));
    }

}

void runMusic() {
  int deltaMax, deltaMin, samplesSum, deltaSums, averageReading, delta, averageDelta;
  int limitMax = 4096;
  int limitMin = 0;
  int scaleIndex = 0;

  int sensorReading = analogRead(MIC_PIN);

  if (sensorReading > limitMax) sensorReading = limitMax;
  if (sensorReading < limitMin) sensorReading = limitMin;

  samples[readingsCounter] = sensorReading;                      // Save sample for dynamic leveling
  if(++readingsCounter >= SAMPLE_SIZE) readingsCounter = 0; // Advance/rollover sample counter
  // Get volume range of prior frames
  samplesSum = 0;
  for(int i=1; i<SAMPLE_SIZE; i++) {
    samplesSum += samples[i];
  }
  averageReading = samplesSum / SAMPLE_SIZE;
  delta = abs(averageReading - sensorReading);
  deltaSamples[readingsCounter] = delta;                      // Save sample for dynamic leveling

  deltaMin = deltaMax = deltaSamples[0];
  for(int i=1; i<SAMPLE_SIZE; i++) {
    if(deltaSamples[i] < deltaMin)      deltaMin = deltaSamples[i];
    else if(deltaSamples[i] > deltaMax) deltaMax = deltaSamples[i];
    deltaSums += delta;
  }

  averageDelta = deltaSums / SAMPLE_SIZE;
  Serial.println(delta);
  if (delta>averageDelta*1.075 || delta<averageDelta*0.925) {
    if (delta>deltaMin*1.05 && delta<deltaMax*1.95) {
      scaleIndex = map(delta, deltaMin, deltaMax*0.8, 0, 17);

      for (int i =0; i<NUM_LEDS; i++) {
        leds[i] = equalizerColorSchemes[scaleIndex][i];
      }
      FastLED.show();
    }
  }


  // delay(100);
}

void runTemperature() {
  int position = abs(temperatureGradientMinLimit - temperature);
  int heatIndex = position * ((255/temperatureGradientMaxLimit-temperatureGradientMinLimit));
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = ColorFromPalette( heatPalette, heatIndex, brightness, currentBlending);
  }
  FastLED.show();
}

void runRainbow() {
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; /* motion speed */

  FillLEDsFromPaletteColors( startIndex);

  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);

  ESPUI.updateSwitcher("Status LEDs", true);
}

void turnOnLED() {
  LAMP_MODE = COLOR_CONTROL_MODE;
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i].setRGB(R, G, B);
  }
  FastLED.show();
  Serial.println("LED ON");
}

void turnOffLED() {
  LAMP_MODE = OFF;
  ESPUI.updateSwitcher("Rainbow animation", false);
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  Serial.println("LED OFF");
}

void showColor(int R, int G, int B) {
}

void processBrightness() {
    FastLED.setBrightness((int) brightness*2.55 );
    FastLED.show();
}

void nextColorPalette() {
  int sizeOfColoPalette = sizeof(colorPalettesNames)/sizeof(*colorPalettesNames);
  colorPaletteIndex = (colorPaletteIndex < sizeOfColoPalette) ? (colorPaletteIndex + 1) : 0;
  colorPaletteName= colorPalettesNames[colorPaletteIndex];
  currentPalette = colorPalettes[colorPaletteIndex];

}

void previousColorPalette() {
  int sizeOfColoPalette = sizeof(colorPalettesNames)/sizeof(*colorPalettesNames);
  Serial.println(sizeOfColoPalette);
  Serial.println(colorPaletteIndex);
  colorPaletteIndex = (colorPaletteIndex > 0) ? (colorPaletteIndex-1) : (sizeOfColoPalette - 1);
  Serial.println(colorPaletteIndex);
  colorPaletteName= colorPalettesNames[colorPaletteIndex];
  currentPalette = colorPalettes[colorPaletteIndex];
  Serial.println(colorPaletteName);
}

void colorPalettePad(Control sender, int value) {
  switch (value) {
  case P_LEFT_UP:
    Serial.print("left up");
    previousColorPalette();
    break;
  case P_RIGHT_UP:
    Serial.println("right up");
    nextColorPalette();
    break;
  case P_FOR_UP:
    Serial.print("for up");
    nextColorPalette();
    break;
  case P_BACK_UP:
    Serial.print("back up");
    previousColorPalette();
    break;
}
Serial.print(" ");
Serial.println(sender.id);
}

void sliderBrightness(Control sender, int type) {
  Serial.println("Brightness set to: ");
  Serial.println((int) sender.value.toInt()*2.55);
  brightness = sender.value.toInt();
  processBrightness();
}

void sliderRGBR(Control sender, int type) {
  ESPUI.updateSwitcher("Rainbow animation", false);
  LAMP_MODE = COLOR_CONTROL_MODE;
  R = (int) sender.value.toInt()*2.55;
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i].setRGB(R, G, B);
  }
  FastLED.show();
  Serial.println(sender.value);
}

void sliderRGBG(Control sender, int type) {
  ESPUI.updateSwitcher("Rainbow animation", false);
  LAMP_MODE = COLOR_CONTROL_MODE;
  G = (int) sender.value.toInt()*2.55;
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i].setRGB(R, G, B);
  }
  FastLED.show();
  Serial.println(sender.value);
}

void sliderRGBB(Control sender, int type) {
  ESPUI.updateSwitcher("Rainbow animation", false);
  LAMP_MODE = COLOR_CONTROL_MODE;
  B = (int) sender.value.toInt()*2.55;
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i].setRGB(R, G, B);
  }
  FastLED.show();
  Serial.println(sender.value);
}

void sliderRainbowSpeed(Control sender, int type) {
  UPDATES_PER_SECOND = sender.value.toInt();
}

void switchSystem(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("System Active");
      break;
    case S_INACTIVE:
      enterDeepSleep();
      Serial.print("System Inactive");
      break;
  }
  Serial.print(" ");
  Serial.println(sender.id);
}

void switchLED(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      turnOnLED();
      Serial.println("LED Active");
      break;
    case S_INACTIVE:
      turnOffLED();
      Serial.println("LED Inactive");
      break;
  }
  Serial.print(" ");
}

void switchRainbow(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Rainbow Active:");
      LAMP_MODE = RAINBOW_MODE;
      break;
    case S_INACTIVE:
      Serial.print("Rainbow Inactive");
      LAMP_MODE = COLOR_CONTROL_MODE;
      break;
  }
  Serial.print(" ");
  Serial.println(sender.id);
}

void switchTemperature(Control sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Temperature Active:");
      LAMP_MODE = TEMPERATURE_MODE;
      break;
    case S_INACTIVE:
      Serial.print("Temperature Inactive");
      LAMP_MODE = COLOR_CONTROL_MODE;
      break;
  }
  Serial.print(" ");
  Serial.println(sender.id);
}

void enterDeepSleep() {
  Serial.println("ENTERING SLEEP MODE NOW");
  turnOffLED();
  delay(500);
  esp_deep_sleep_start();
}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;

    for( int i = 0; i < NUM_LEDS; i++) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}
