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

/*--------------------------- JSON builders -----------------*/
void setConfigSchema()
{
  // Define our config schema
  JsonDocument json;

  JsonObject strips = json["strips"].to<JsonObject>();
  strips["type"] = "array";
  
  JsonObject items = strips["items"].to<JsonObject>();
  items["type"] = "object";

  JsonObject properties = items["properties"].to<JsonObject>();

  JsonObject strip = properties["strip"].to<JsonObject>();
  strip["type"] = "integer";
  strip["minimum"] = 1;
  strip["maximum"] = PWM_CHANNEL_COUNT;

  JsonObject type = properties["type"].to<JsonObject>();
  JsonArray typeEnum = type["enum"].to<JsonArray>();
  typeEnum.add("single");
  typeEnum.add("cct");
  typeEnum.add("rgb");
  typeEnum.add("rgbw");
  typeEnum.add("rgbww");

  JsonArray required = items["required"].to<JsonArray>();
  required.add("strip");
  required.add("type");

  JsonObject fadeIntervalUs = json["fadeIntervalUs"].to<JsonObject>();
  fadeIntervalUs["type"] = "integer";
  fadeIntervalUs["minimum"] = 0;

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

  JsonObject properties = colour["properties"].to<JsonObject>();

  JsonObject mired = properties["mired"].to<JsonObject>();
  mired["type"] = "integer";
  mired["minimum"] = 167;
  mired["maximum"] = 500;

  JsonObject red = properties["red"].to<JsonObject>();
  red["type"] = "integer";
  red["minimum"] = 0;
  red["maximum"] = 255;

  JsonObject green = properties["green"].to<JsonObject>();
  green["type"] = "integer";
  green["minimum"] = 0;
  green["maximum"] = 255;

  JsonObject blue = properties["blue"].to<JsonObject>();
  blue["type"] = "integer";
  blue["minimum"] = 0;
  blue["maximum"] = 255;

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
      pwmDriver.crossfade(strip->index, channelOffset, colour[0]);
    }
    else if (strip->channels == 2) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, colour[0], colour[1]);
    }
    else if (strip->channels == 3) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, colour[0], colour[1], colour[2]);
    }
    else if (strip->channels == 4) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, colour[0], colour[1], colour[2], colour[3]);
    }
    else if (strip->channels == 5) 
    {
      pwmDriver.crossfade(strip->index, channelOffset, colour[0], colour[1], colour[2], colour[3], colour[4]);
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

  uint8_t channelOffset = 0;

  for (uint8_t strip = 0; strip < PWM_CHANNEL_COUNT; strip++)
  {
    LEDStrip *ledStrip = &ledStrips[strip];

    if (ledStrip->state == LED_STATE_OFF)
    {
      ledFade(ledStrip, channelOffset, OFF);
    }
    else if (ledStrip->state == LED_STATE_ON)
    {
      float_t brightness_pct = ((float_t)ledStrip->brightness / (float_t)255);
    
      uint8_t colour[MAX_LED_COUNT];

      for (uint8_t i = 0; i < ledStrip->channels; i++) 
      {
        colour[i] = ledStrip->colour[i] * brightness_pct;
      }

      ledFade(ledStrip, channelOffset, colour);
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
  if (strcmp(type, "single") == 0) {
    ledStrip->channels = 1;
  } else if (strcmp(type, "cct") == 0) {
    ledStrip->channels = 2;
  } else if (strcmp(type, "rgb") == 0) {
    ledStrip->channels = 3;
  } else if (strcmp(type, "rgbw") == 0) {
    ledStrip->channels = 4;
  } else if (strcmp(type, "rgbww") == 0) {
    ledStrip->channels = 5;  
  }

  ledStrip->state = LED_STATE_OFF;
  ledStrip->brightness = 255;

  // if only a single channel control is via brightness only
  if (ledStrip->channels == 1) 
  {
    ledStrip->colour[0] = 255;
  } else 
  {
    for (uint8_t i = 0; i < ledStrip->channels; i++)
    {
      ledStrip->colour[i] = 0;
    }
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
      uint16_t mired = colour["mired"].as<uint16_t>();
      float_t kelvin = 1000000L / mired;

      // CW range is 2500-6000K
      // WW range is 2000-5500K
      if (kelvin < 2000) kelvin = 2000L;
      if (kelvin > 6000) kelvin = 6000L;

      uint8_t cw = 0;
      if (kelvin > 2500)
      {
        uint8_t pwm = ((kelvin - 2500L) / 3500L) * 255;
        cw = pwm;
      }

      uint8_t ww = 0;
      if (kelvin < 5500)
      {
        uint8_t pwm = ((kelvin - 2000L) / 3500L) * 255;
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
}

void jsonCommand(JsonVariant json)
{
  if (json.containsKey("strip"))
  {
    jsonStripCommand(json);
  }
}

/*--------------------------- Program -------------------------------*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);  
  Serial.println(F("[light] starting up..."));

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
}