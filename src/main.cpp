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
#include <ledPWM.h>                   // For PWM LED controller
#include <OXRS_HASS.h>                // For Home Assistant self-discovery

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

#define MIN_PWM                     0
#define MAX_PWM                     255
#define MIN_MIREDS                  167
#define MAX_MIREDS                  500

/*-------------------------- Internal datatypes --------------------------*/
struct LEDStrip
{
  uint8_t index;
  uint8_t channels;

  uint8_t state;
  uint8_t brightness;
  uint8_t color[MAX_LED_COUNT];
  uint16_t mired;

  uint32_t lastFadeUs;
  bool publishState;
  bool publishHassDiscovery;
};

/*--------------------------- Global Variables ---------------------------*/
// Fade interval used if no explicit interval defined in command payload
uint32_t g_fade_interval_us = DEFAULT_FADE_INTERVAL_US;

// PWM LED controllers
PWMDriver pwmDriver;

// LED strip config (allow for a max of all single LED strips)
LEDStrip ledStrips[PWM_CHANNEL_COUNT];

/*--------------------------- Instantiate Globals ---------------------*/
// Home Assistant self-discovery
OXRS_HASS hass(oxrs.getMQTT());

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

  // Add any Home Assistant config
  hass.setConfigSchema(json);

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
  brightness["minimum"] = MIN_PWM;
  brightness["maximum"] = MAX_PWM;

  JsonObject color_temp = json["color_temp"].to<JsonObject>();
  color_temp["type"] = "integer";
  color_temp["minimum"] = MIN_MIREDS;
  color_temp["maximum"] = MAX_MIREDS;

  JsonObject color = json["color"].to<JsonObject>();
  color["type"] = "object";

  JsonObject properties = color["properties"].to<JsonObject>();

  JsonObject r = properties["r"].to<JsonObject>();
  r["type"] = "integer";
  r["minimum"] = MIN_PWM;
  r["maximum"] = MAX_PWM;

  JsonObject g = properties["g"].to<JsonObject>();
  g["type"] = "integer";
  g["minimum"] = MIN_PWM;
  g["maximum"] = MAX_PWM;

  JsonObject b = properties["b"].to<JsonObject>();
  b["type"] = "integer";
  b["minimum"] = MIN_PWM;
  b["maximum"] = MAX_PWM;

  JsonObject c = properties["c"].to<JsonObject>();
  c["type"] = "integer";
  c["minimum"] = MIN_PWM;
  c["maximum"] = MAX_PWM;

  JsonObject w = properties["w"].to<JsonObject>();
  w["type"] = "integer";
  w["minimum"] = MIN_PWM;
  w["maximum"] = MAX_PWM;

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
void calculateColorTemp(LEDStrip * ledStrip)
{
  // only need to calculate colour temp for cct and rgbww strips
  if (ledStrip->channels != 2 && ledStrip->channels != 5)
    return;

  float_t kelvin = 1000000L / ledStrip->mired;

  // CW range is 2500-6000K
  // WW range is 2000-5500K
  if (kelvin < 2000) kelvin = 2000L;
  if (kelvin > 6000) kelvin = 6000L;

  uint8_t cw = MIN_PWM;
  if (kelvin > 2500)
  {
    uint8_t pwm = ((kelvin - 2500L) / 3500L) * MAX_PWM;
    cw = pwm;
  }

  uint8_t ww = MIN_PWM;
  if (kelvin < 5500)
  {
    uint8_t pwm = ((kelvin - 2000L) / 3500L) * MAX_PWM;
    ww = MAX_PWM - pwm;
  }

  if (ledStrip->channels == 2) 
  {
    ledStrip->color[0] = cw;
    ledStrip->color[1] = ww;
  }
  else
  {
    ledStrip->color[3] = cw;
    ledStrip->color[4] = ww;
  }
}

void publishHassDiscovery(LEDStrip * ledStrip)
{
  uint8_t strip = ledStrip->index + 1;

  char component[8];
  sprintf_P(component, PSTR("light"));

  char stripId[16];
  sprintf_P(stripId, PSTR("led_strip_%d"), strip);

  char stripName[16];
  sprintf_P(stripName, PSTR("LED Strip %d"), strip);

  char mqttTopic[64];
  char mqttTemplate[512];

  // JSON config payload (empty if the input is disabled, to clear any existing config)
  JsonDocument json;

  hass.getDiscoveryJson(json, stripId);

  json["name"] = stripName;
  json["schema"] = "template";
  json["stat_t"] = oxrs.getMQTT()->getStatusTopic(mqttTopic);
  json["cmd_t"] = oxrs.getMQTT()->getCommandTopic(mqttTopic);

  sprintf_P(mqttTemplate, PSTR("{'strip':%d,'state':'off'}"), strip);
  json["cmd_off_tpl"] = mqttTemplate;

  sprintf_P(mqttTemplate, PSTR("{'strip':%d,'state':'on'{%% if brightness is defined %%},'brightness':{{ brightness }}{%% endif %%}{%% if color_temp is defined %%},'color_temp':{{ color_temp }}{%% endif %%}{%% if red is defined and green is defined and blue is defined %%},'color':{'r':{{ red }},'g':{{ green }},'b':{{ blue }}}{%% endif %%}}"), strip);
  json["cmd_on_tpl"] = mqttTemplate;

  sprintf_P(mqttTemplate, PSTR("{%% if value_json.strip == %d %%}{{ value_json.state }}{%% endif %%}"), strip);
  json["stat_tpl"] = mqttTemplate;

  sprintf_P(mqttTemplate, PSTR("{%% if value_json.strip == %d %%}{{ value_json.brightness }}{%% endif %%}"), strip);
  json["bri_tpl"] = mqttTemplate;

  if (ledStrip->channels == 2 || ledStrip->channels == 5)
  {
    sprintf_P(mqttTemplate, PSTR("{%% if value_json.strip == %d %%}{{ value_json.color_temp }}{%% endif %%}"), strip);
    json["clr_temp_tpl"] = mqttTemplate;

    json["max_mirs"] = MAX_MIREDS;
    json["min_mirs"] = MIN_MIREDS;
  }
  
  if (ledStrip->channels >= 3)
  {
    sprintf_P(mqttTemplate, PSTR("{%% if value_json.strip == %d %%}{{ value_json.color.r }}{%% endif %%}"), strip);
    json["r_tpl"] = mqttTemplate;
    sprintf_P(mqttTemplate, PSTR("{%% if value_json.strip == %d %%}{{ value_json.color.g }}{%% endif %%}"), strip);
    json["g_tpl"] = mqttTemplate;
    sprintf_P(mqttTemplate, PSTR("{%% if value_json.strip == %d %%}{{ value_json.color.b }}{%% endif %%}"), strip);
    json["b_tpl"] = mqttTemplate;
  }

  // Publish retained and stop trying once successful 
  hass.publishDiscoveryJson(json, component, stripId);
}

void publishStripStatus(LEDStrip * ledStrip)
{
  JsonDocument json;

  json["strip"] = ledStrip->index + 1;
  json["state"] = ledStrip->state == LED_STATE_ON ? "on" : "off";
  json["brightness"] = ledStrip->brightness;

  switch (ledStrip->channels)
  {
    case 2:
      json["color_temp"] = ledStrip->mired;
      break;

    case 3:
      json["color"]["r"] = ledStrip->color[0];
      json["color"]["g"] = ledStrip->color[1];
      json["color"]["b"] = ledStrip->color[2];
      break;

    case 4:
      json["color"]["r"] = ledStrip->color[0];
      json["color"]["g"] = ledStrip->color[1];
      json["color"]["b"] = ledStrip->color[2];
      json["color"]["w"] = ledStrip->color[3];
    break;
    
    case 5:
      json["color"]["r"] = ledStrip->color[0];
      json["color"]["g"] = ledStrip->color[1];
      json["color"]["b"] = ledStrip->color[2];
      json["color"]["c"] = ledStrip->color[3];
      json["color"]["w"] = ledStrip->color[4];
      break;
  }

  oxrs.publishStatus(json);
}

void ledFade(LEDStrip * ledStrip, uint8_t channelOffset, uint8_t color[])
{
  if ((micros() - ledStrip->lastFadeUs) > g_fade_interval_us)
  {
    if (ledStrip->channels == 1) 
    {
      pwmDriver.crossfade(ledStrip->index, channelOffset, color[0]);
    }
    else if (ledStrip->channels == 2) 
    {
      pwmDriver.crossfade(ledStrip->index, channelOffset, color[0], color[1]);
    }
    else if (ledStrip->channels == 3) 
    {
      pwmDriver.crossfade(ledStrip->index, channelOffset, color[0], color[1], color[2]);
    }
    else if (ledStrip->channels == 4) 
    {
      pwmDriver.crossfade(ledStrip->index, channelOffset, color[0], color[1], color[2], color[3]);
    }
    else if (ledStrip->channels == 5) 
    {
      pwmDriver.crossfade(ledStrip->index, channelOffset, color[0], color[1], color[2], color[3], color[4]);
    }  

    ledStrip->lastFadeUs = micros();

    // if we have completed a fade, publish the state
    if (ledStrip->publishState && pwmDriver.fadeComplete[ledStrip->index])
    {
      publishStripStatus(ledStrip);

      ledStrip->publishState = false;
    }
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
    ledStrip->brightness = MIN_PWM;

    for (uint8_t i = 0; i < MAX_LED_COUNT; i++)
    {
      ledStrip->color[i] = MIN_PWM;
    }

    ledStrip->lastFadeUs = 0L;

    ledStrip->publishState = false;
    ledStrip->publishHassDiscovery = true;
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

    // ignore if the strip is not configured
    if (ledStrip->channels == 0)
      continue;;

    if (ledStrip->state == LED_STATE_OFF)
    {
      ledFade(ledStrip, channelOffset, OFF);
    }
    else if (ledStrip->state == LED_STATE_ON)
    {
      calculateColorTemp(ledStrip);

      float_t brightness_pct = ((float_t)ledStrip->brightness / (float_t)MAX_PWM);
    
      uint8_t color[MAX_LED_COUNT];

      for (uint8_t i = 0; i < ledStrip->channels; i++) 
      {
        color[i] = ledStrip->color[i] * brightness_pct;
      }

      ledFade(ledStrip, channelOffset, color);
    }

    // increase offset
    channelOffset += ledStrip->channels;
  }
}

void publishHassDiscovery()
{
  for (uint8_t strip = 0; strip < PWM_CHANNEL_COUNT; strip++)
  {
    LEDStrip *ledStrip = &ledStrips[strip];

    // ignore if the strip is not configured
    if (ledStrip->channels == 0)
      continue;;

    if (ledStrip->publishHassDiscovery)
    {
      publishHassDiscovery(ledStrip);

      ledStrip->publishHassDiscovery = false;
    }
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
  ledStrip->brightness = MAX_PWM;
  ledStrip->mired = MIN_MIREDS;

  // if only a single channel control is via brightness only
  if (ledStrip->channels == 1) 
  {
    ledStrip->color[0] = MAX_PWM;
  } 
  else
  {
    for (uint8_t i = 0; i < ledStrip->channels; i++)
    {
      ledStrip->color[i] = MIN_PWM;
    }
  }

  // republish the HA discovery payload if the config changes
  ledStrip->publishHassDiscovery = true;
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
      ledStrip->publishState = true;
    }
    else if (strcmp(json["state"], "off") == 0)
    {
      ledStrip->state = LED_STATE_OFF;
      ledStrip->publishState = true;
    }
    else 
    {
      oxrs.println(F("[light] invalid state"));
    }
  }

  if (json.containsKey("brightness"))
  {
    ledStrip->brightness = json["brightness"].as<uint8_t>();
    ledStrip->publishState = true;
  }

  if (json.containsKey("color_temp"))
  {    
    ledStrip->mired = json["color_temp"].as<uint16_t>();
    ledStrip->publishState = true;
  }

  if (json.containsKey("color"))
  {
    JsonObject color = json["color"].as<JsonObject>();

    if (color.containsKey("r"))
    {
      ledStrip->color[0] = color["r"].as<uint8_t>();
    }

    if (color.containsKey("g"))
    {
      ledStrip->color[1] = color["g"].as<uint8_t>();
    }

    if (color.containsKey("b"))
    {
      ledStrip->color[2] = color["b"].as<uint8_t>();
    }

    if (color.containsKey("c"))
    {
      ledStrip->color[3] = color["c"].as<uint8_t>();
    }

    if (color.containsKey("w"))
    {
      ledStrip->color[4] = color["w"].as<uint8_t>();
    }

    ledStrip->publishState = true;
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

  // Handle any Home Assistant config
  hass.parseConfig(json);
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

  // Check if we need to publish any Home Assistant discovery payloads
  if (hass.isDiscoveryEnabled())
  {
    publishHassDiscovery();
  }
}