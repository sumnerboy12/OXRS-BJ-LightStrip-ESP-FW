/**
  LED light strip controller firmware for the Open eXtensible Rack System

  Documentation:  
    https://oxrs.io/docs/hardware/output-devices/light-strip-controller.html
  
  GitHub repository:
    https://github.com/sumnerboy12/OXRS-BJ-LightStrip-ESP-FW

  Copyright 2025 Ben Jones
*/

/*------------------------- PWM Type ----------------------------------*/
//#define PCA_MODE          // 16ch PCA9865 PWM controllers (I2C)
//#define GPIO_PWM1-5       // 5ch GPIO MOSFETs

/*--------------------------- Libraries -------------------------------*/
#include <Arduino.h>
#include <ledPWM.h>                 // For PWM LED controller
#include <OXRS_SENSORS.h>           // For QWICC I2C sensors

#if defined(OXRS_ESP32)
#include <OXRS_32.h>                  // ESP32 support
OXRS_32 oxrs;

#elif defined(OXRS_ESP8266)
#include <OXRS_8266.h>                // ESP8266 support
OXRS_8266 oxrs;

#elif defined(OXRS_LILYGO)
#include <OXRS_LILYGOPOE.h>           // LilyGO T-ETH-POE support
OXRS_LILYGOPOE oxrs;
#endif

/*--------------------------- Constants ----------------------------------*/
// Serial
#define SERIAL_BAUD_RATE            115200

// Only support up to 5 LEDs per strip
#define MAX_LED_COUNT               5

// Supported LED states
#define LED_STATE_OFF               0
#define LED_STATE_ON                1

// Default fade interval (microseconds)
#define DEFAULT_FADE_INTERVAL_US    500L;

// Only 5 GPIO channels
#define PWM_CHANNEL_COUNT           5

/*-------------------------- Internal datatypes --------------------------*/
struct LEDStrip
{
  uint8_t index;
  uint8_t channels;

  uint8_t state;
  uint8_t brightness;
  uint8_t colour[MAX_LED_COUNT];

  uint32_t fadeIntervalUs;
  uint32_t lastFadeUs;
};

/*--------------------------- Global Variables ---------------------------*/
// Fade interval used if no explicit interval defined in command payload
uint32_t g_fade_interval_us = DEFAULT_FADE_INTERVAL_US;

// PWM LED controllers
PWMDriver pwmDriver;

// LED strip config (allow for a max of all single LED strips)
LEDStrip ledStrips[PWM_CHANNEL_COUNT];

// I2C sensors
OXRS_SENSORS sensors;

/*--------------------------- JSON builders -----------------*/
void setConfigSchema()
{
  // Define our config schema
  JsonDocument json;

  JsonObject strips = json["strips"].to<JsonObject>();
  strips["type"] = "array";
  strips["description"] = "Define what strips are connected where";
  
  JsonObject stripItems = strips["items"].to<JsonObject>();
  stripItems["type"] = "object";

  JsonObject stripProperties = stripItems["properties"].to<JsonObject>();

  JsonObject strip = stripProperties["strip"].to<JsonObject>();
  strip["type"] = "integer";
  strip["description"] = "Assigns an index to the strip, incrementing from 1, in the order it is wired to the controller";
  strip["minimum"] = 1;
  strip["maximum"] = PWM_CHANNEL_COUNT;

  JsonObject type = stripProperties["type"].to<JsonObject>();
  type["type"] = "string";
  type["description"] = "Type of strip, defines the number of channels for this strip";
  JsonArray typeEnum = type["enum"].to<JsonArray>();
  typeEnum.add("single");
  typeEnum.add("cct");
  typeEnum.add("rgb");
  typeEnum.add("rgbw");
  typeEnum.add("rgbww");

  JsonArray required = stripItems["required"].to<JsonArray>();
  required.add("strip");
  required.add("type");

  JsonObject fadeIntervalUs = json["fadeIntervalUs"].to<JsonObject>();
  fadeIntervalUs["type"] = "integer";
  fadeIntervalUs["minimum"] = 0;
  fadeIntervalUs["description"] = "Default time to fade from off -> on (and vice versa), in microseconds (defaults to 500us)";

  // Add any sensor config
  sensors.setConfigSchema(json);

  // Pass our config schema down to the hardware library
  oxrs.setConfigSchema(json.as<JsonVariant>());
}

void setCommandSchema()
{
  // Define our command schema
  JsonDocument json;
  
  JsonObject strip = json["strip"].to<JsonObject>();
  strip["type"] = "integer";
  strip["minimum"] = 1;
  strip["maximum"] = PWM_CHANNEL_COUNT;

  JsonObject state = json["state"].to<JsonObject>();
  state["type"] = "string";
  JsonArray stateEnum = state["enum"].to<JsonArray>();
  stateEnum.add("on");
  stateEnum.add("off");

  JsonObject brightness = json["brightness"].to<JsonObject>();
  brightness["type"] = "integer";
  brightness["minimum"] = 0;
  brightness["maximum"] = 255;

  JsonObject colour = json["colour"].to<JsonObject>();
  colour["type"] = "object";

  JsonObject colourProperties = colour["properties"].to<JsonObject>();

  JsonObject mired = colourProperties["mired"].to<JsonObject>();
  mired["type"] = "integer";
  mired["minimum"] = 167;
  mired["maximum"] = 500;

  JsonObject red = colourProperties["red"].to<JsonObject>();
  red["type"] = "integer";
  red["minimum"] = 0;
  red["maximum"] = 255;

  JsonObject green = colourProperties["green"].to<JsonObject>();
  green["type"] = "integer";
  green["minimum"] = 0;
  green["maximum"] = 255;

  JsonObject blue = colourProperties["blue"].to<JsonObject>();
  blue["type"] = "integer";
  blue["minimum"] = 0;
  blue["maximum"] = 255;

  JsonObject fadeIntervalUs = json["fadeIntervalUs"].to<JsonObject>();
  fadeIntervalUs["type"] = "integer";
  fadeIntervalUs["minimum"] = 0;

  JsonObject restart = json["restart"].to<JsonObject>();
  restart["type"] = "boolean";
  restart["description"] = "Restart the controller";

  // Add any sensor commands
  sensors.setCommandSchema(json);

  // Pass our command schema down to the hardware library
  oxrs.setCommandSchema(json.as<JsonVariant>());
}

void initialisePwmDrivers()
{
  oxrs.println(F("[light] using direct PWM control via GPIOs..."));

  oxrs.print(F("[light]  - GPIO_PWM1 -> "));
  oxrs.println(GPIO_PWM1);
  oxrs.print(F("[light]  - GPIO_PWM2 -> "));
  oxrs.println(GPIO_PWM2);
  oxrs.print(F("[light]  - GPIO_PWM3 -> "));
  oxrs.println(GPIO_PWM3);
  oxrs.print(F("[light]  - GPIO_PWM4 -> "));
  oxrs.println(GPIO_PWM4);
  oxrs.print(F("[light]  - GPIO_PWM5 -> "));
  oxrs.println(GPIO_PWM5);

  // Initialise the direct PWM for this address
  pwmDriver.begin_gpio(GPIO_PWM1, GPIO_PWM2, GPIO_PWM3, GPIO_PWM4, GPIO_PWM5);
}

/*--------------------------- LED -----------------*/
void ledFade(LEDStrip * strip, uint8_t channelOffset, uint8_t colour[])
{
  if ((micros() - strip->lastFadeUs) > strip->fadeIntervalUs)
  {    
    if (strip->channels == 1) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, strip->brightness * colour[0]);
    }
    else if (strip->channels == 2) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, strip->brightness * colour[0], strip->brightness * colour[1]);
    }
    else if (strip->channels == 3) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, strip->brightness * colour[0], strip->brightness * colour[1], strip->brightness * colour[2]);
    }
    else if (strip->channels == 4) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, strip->brightness * colour[0], strip->brightness * colour[1], strip->brightness * colour[2], strip->brightness * colour[3]);
    }
    else if (strip->channels == 5) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, strip->brightness * colour[0], strip->brightness * colour[1], strip->brightness * colour[2], strip->brightness * colour[3], strip->brightness * colour[4]);
    }  

    strip->lastFadeUs = micros();
  }
}

void initialiseStrips()
{
  for (uint8_t strip = 0; strip < PWM_CHANNEL_COUNT; strip++)
  {
    LEDStrip * ledStrip = &ledStrips[strip];

    // .index is immutable and shouldn't be changed
    ledStrip->index = strip;
    ledStrip->channels = 0;
    ledStrip->state = LED_STATE_OFF;
    ledStrip->brightness = 0;

    for (uint8_t i = 0; i < MAX_LED_COUNT; i++)
    {
      ledStrip->colour[i] = 0;
    }

    ledStrip->fadeIntervalUs = g_fade_interval_us; 
    ledStrip->lastFadeUs = 0L;
  }
}

void processStrips()
{
  uint8_t OFF[MAX_LED_COUNT];
  memset(OFF, 0, sizeof(OFF));
  uint16_t pcaState = 0x0;

  uint8_t channelOffset = 0;

  for (uint8_t strip = 0; strip < PWM_CHANNEL_COUNT; strip++)
  {
    LEDStrip *ledStrip = &ledStrips[strip];

    if (ledStrip->state == LED_STATE_OFF)
    {
      // off
      ledFade(ledStrip, channelOffset, OFF);
    }
    else if (ledStrip->state == LED_STATE_ON)
    {
      // fade
      ledFade(ledStrip, channelOffset, ledStrip->colour);
    }

    // increase offset
    channelOffset += ledStrip->channels;
  }
}

uint8_t getStrip(JsonVariant json)
{
  if (!json.containsKey("strip"))
  {
    oxrs.println(F("[light] missing strip"));
    return 0;
  }
  
  uint8_t strip = json["strip"].as<uint8_t>();

  // Check the strip is valid for this device
  if (strip <= 0 || strip > PWM_CHANNEL_COUNT)
  {
    oxrs.println(F("[light] invalid strip"));
    return 0;
  }

  return strip;
}

void jsonStripConfig(JsonVariant json)
{
  uint8_t strip = getStrip(json);
  if (strip == 0) return;

  // strip index is sent 1-based
  LEDStrip * ledStrip = &ledStrips[strip - 1];

  const char * type = json["type"].as<const char *>();
  if (strcmp(type, "single")) {
    ledStrip->channels = 1;
  } else if (strcmp(type, "cct")) {
    ledStrip->channels = 2;
  } else if (strcmp(type, "rgb")) {
    ledStrip->channels = 3;
  } else if (strcmp(type, "rgbw")) {
    ledStrip->channels = 4;
  } else if (strcmp(type, "rgbww")) {
    ledStrip->channels = 5;  
  }

  ledStrip->state = LED_STATE_OFF;
  ledStrip->brightness = 0;

  for (uint8_t i = 0; i < MAX_LED_COUNT; i++)
  {
    ledStrip->colour[i] = 0;
  }
}

void jsonStripCommand(JsonVariant json)
{
  uint8_t strip = getStrip(json);
  if (strip == 0) return;

  // strip index is sent 1-based
  LEDStrip * ledStrip = &ledStrips[strip - 1];
  if (ledStrip->channels == 0) return;

  if (json.containsKey("state"))
  {
    if (strcmp(json["state"], "on") == 0)
    {
      ledStrip->state = LED_STATE_ON;
    }
    else if (strcmp(json["state"], "off") == 0)
    {
      ledStrip->state = LED_STATE_OFF;
    }
    else 
    {
      oxrs.println(F("[light] invalid state"));
    }
  }

  if (json.containsKey("brightness"))
  {
    ledStrip->brightness = json["brightness"].as<uint8_t>();
  }

  if (json.containsKey("colour"))
  {
    JsonObject colour = json["colour"].as<JsonObject>();
    
    if (colour.containsKey("red"))
    {
      ledStrip->colour[0] = colour["red"].as<uint8_t>();
    }

    if (colour.containsKey("green"))
    {
      ledStrip->colour[1] = colour["green"].as<uint8_t>();
    }

    if (colour.containsKey("blue"))
    {
      ledStrip->colour[2] = colour["blue"].as<uint8_t>();
    }

    if (colour.containsKey("mired"))
    {
      uint8_t mired = colour["mired"].as<uint8_t>();
      uint8_t kelvin = 1000000 / mired;

      // CW range is 2500-6000K
      // WW range is 2000-5500K
      if (kelvin < 2000) kelvin = 2000;
      if (kelvin > 6000) kelvin = 6000;

      uint8_t cw = 0;
      if (kelvin > 2500)
      {
        uint8_t pwm = ((float)(kelvin - 2500) / (float)3500) * 255;
        cw = pwm;
      }

      uint8_t ww = 0;
      if (kelvin < 5500)
      {
        uint8_t pwm = ((float)(kelvin - 2000) / (float)3500) * 255;
        ww = 255 - pwm;
      }

      if (ledStrip->channels == 2) 
      {
        ledStrip->colour[0] = cw;
        ledStrip->colour[1] = ww;
      }
      else
      {
        ledStrip->colour[3] = cw;
        ledStrip->colour[4] = ww;
      }
    }
  }

  if (json.containsKey("fadeIntervalUs"))
  {
    ledStrip->fadeIntervalUs = json["fadeIntervalUs"].as<uint32_t>();
  }
  else
  {
    ledStrip->fadeIntervalUs = g_fade_interval_us;
  }
}

void jsonConfig(JsonVariant json)
{
  if (json.containsKey("strips"))
  {
    for (JsonVariant strip : json["strips"].as<JsonArray>())
    {
      jsonStripConfig(strip);
    }
  }

  if (json.containsKey("fadeIntervalUs"))
  {
    g_fade_interval_us = json["fadeIntervalUs"].as<uint32_t>();
  }

  // Let the sensors handle any config
  sensors.conf(json);
}

void jsonCommand(JsonVariant json)
{
  if (json.containsKey("strip"))
  {
    jsonStripCommand(json);
  }

  if (json.containsKey("restart") && json["restart"].as<bool>())
  {
    ESP.restart();
  }

  // Let the sensors handle any commands
  sensors.cmnd(json);
}

/*--------------------------- Program -------------------------------*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);  
  Serial.println(F("[light] starting up..."));

  // Start the I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Start the sensor library (scan for attached sensors)
  sensors.begin();

  // Initialise PWM drivers
  initialisePwmDrivers();

  // Initialise LED strips
  initialiseStrips();

  // Start hardware
  oxrs.begin(jsonConfig, jsonCommand);

  // Set up the config/command schema (for self-discovery and adoption)
  setConfigSchema();
  setCommandSchema();
}

void loop()
{
  // Let hardware handle any events etc
  oxrs.loop();

  // Process any PWM updates
  processStrips();

  // Publish sensor telemetry (if any)
  JsonDocument telemetry;
  sensors.tele(telemetry.as<JsonVariant>());

  if (telemetry.size() > 0)
  {
    oxrs.publishTelemetry(telemetry.as<JsonVariant>());
  }
}