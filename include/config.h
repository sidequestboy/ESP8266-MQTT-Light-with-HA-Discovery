/*
 * This is a sample configuration file for the "mqtt_esp8266" light.
 *
 * Change the settings below and save the file as "config.h"
 * You can then upload the code using the Arduino IDE.
 */

// Leave this here. These are the choices for CONFIG_STRIP below.
enum strip {
  BRIGHTNESS, // only one color/only white
  RGB,        // RGB LEDs
  RGBW        // RGB LEDs with an extra white LED per LED
};

#define CONFIG_STRIP RGB // Choose one of the options from above.

// Pins
// In case of BRIGHTNESS: only WHITE is used
// In case of RGB(W): red, green, blue(, white) is used
// All values need to be present, if they are not needed, set to -1,
// it will be ignored.
#define CONFIG_PIN_RED   14  // For RGB(W)
#define CONFIG_PIN_GREEN 12  // For RGB(W)
#define CONFIG_PIN_BLUE  13  // For RGB(W)
#define CONFIG_PIN_WHITE -1 // For BRIGHTNESS and RGBW

// WiFi
#define CONFIG_WIFI_SSID "WhyFi"
#define CONFIG_WIFI_PASS "oatsforbreakfast"

// MQTT
#define CONFIG_MQTT_HOST "homeassistant.lan"
#define CONFIG_MQTT_PORT 1883 // Usually 1883
#define CONFIG_MQTT_USER "mqtt"
#define CONFIG_MQTT_PASS "mqtt"
#define CONFIG_MQTT_CLIENT_ID "jamie_s_bedroom_rgb_led_strip" // Must be unique on the MQTT network
#define CONFIG_MQTT_NAME "Jamie's Bedroom RGB LED Strip"

#define CONFIG_DEFAULT_TRANSITION_TIME 1

// MQTT Topics
#define CONFIG_MQTT_BASE_TOPIC             "homeassistant/light/jamie_s_bedroom_rgb_led_strip"
#define CONFIG_MQTT_TOPIC_STATE            "homeassistant/light/jamie_s_bedroom_rgb_led_strip/light/status"
#define CONFIG_MQTT_TOPIC_SET              "homeassistant/light/jamie_s_bedroom_rgb_led_strip/light/switch"
#define CONFIG_MQTT_TOPIC_BRIGHTNESS_SET   "homeassistant/light/jamie_s_bedroom_rgb_led_strip/brightness/set"
#define CONFIG_MQTT_TOPIC_RGB_SET          "homeassistant/light/jamie_s_bedroom_rgb_led_strip/rgb/set"
#define CONFIG_MQTT_TOPIC_DISCOVERY        "homeassistant/light/jamie_s_bedroom_rgb_led_strip/config"
#define CONFIG_MQTT_STATE_VALUE_TEMPLATE "{{ value_json.state }}"
#define CONFIG_MQTT_BRIGHTNESS_VALUE_TEMPLATE "{{ value_json.brightness }}"
#define CONFIG_MQTT_RGB_VALUE_TEMPLATE "{{ value_json.rgb | join(',') }}"

#define CONFIG_MQTT_PAYLOAD_ON "ON"
#define CONFIG_MQTT_PAYLOAD_OFF "OFF"

// Miscellaneous
// Default number of flashes if no value was given
#define CONFIG_DEFAULT_FLASH_LENGTH 2
// Number of seconds for one transition in colorfade mode
#define CONFIG_COLORFADE_TIME_SLOW 10
#define CONFIG_COLORFADE_TIME_FAST 3

// Reverse the LED logic
// false: 0 (off) - 255 (bright)
// true: 255 (off) - 0 (bright)
#define CONFIG_INVERT_LED_LOGIC false

// Set the mode for the built-in LED on some boards.
// -1 = Do nothing. Leave the pin in its default state.
//  0 = Explicitly set the LED_BUILTIN to LOW.
//  1 = Explicitly set the LED_BUILTIN to HIGH. (Off for Wemos D1 Mini)
#define CONFIG_LED_BUILTIN_MODE -1

// Enables Serial and print statements
#define CONFIG_DEBUG true
