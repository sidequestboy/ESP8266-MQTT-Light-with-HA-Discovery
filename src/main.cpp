/*
 * ESP8266 MQTT Lights for Home Assistant.
 *
 * Created DIY lights for Home Assistant using MQTT and JSON.
 * This project supports single-color, RGB, and RGBW lights.
 *
 * Copy the included `config-sample.h` file to `config.h` and update
 * accordingly for your setup.
 *
 * Based on corbanmailloux code here:
 * See https://github.com/corbanmailloux/esp-mqtt-rgb-led for more information.
 */

// Set configuration options for LED type, pins, WiFi, and MQTT in the following file:
#include "config.h"

// https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>

#include <ESP8266WiFi.h>

// http://pubsubclient.knolleary.net/
#include <PubSubClient.h>

const bool rgb = (CONFIG_STRIP == RGB) || (CONFIG_STRIP == RGBW);
const bool includeWhite = (CONFIG_STRIP == BRIGHTNESS) || (CONFIG_STRIP == RGBW);

const int BUFFER_SIZE = JSON_OBJECT_SIZE(30);

// Maintained state for reporting to HA
byte red = 255;
byte green = 255;
byte blue = 255;
byte white = 255;
byte brightness = 255;

// Real values to write to the LEDs (ex. including brightness and state)
byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;
byte realWhite = 0;

bool stateOn = false;

// Globals for fade/transitions
bool startFade = false;
unsigned long lastLoop = 0;
unsigned int transitionTime = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB, stepW;
int redVal, grnVal, bluVal, whtVal;

// Globals for flash
bool flash = false;
bool startFlash = false;
unsigned int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashWhite = white;
byte flashBrightness = brightness;

// Globals for colorfade
bool colorfade = false;
int currentColor = 0;
// {red, grn, blu, wht}
const byte colors[][4] = {
  {255, 0, 0, 0},
  {0, 255, 0, 0},
  {0, 0, 255, 0},
  {255, 80, 0, 0},
  {163, 0, 255, 0},
  {0, 255, 255, 0},
  {255, 255, 0, 0}
};
const int numColors = 7;

WiFiClient espClient;
PubSubClient client(espClient);

StaticJsonDocument<BUFFER_SIZE> doc;


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(CONFIG_WIFI_SSID);

  WiFi.mode(WIFI_STA); // Disable the built-in WiFi access point.
  WiFi.begin(CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
bool processJson(char* message) {
  doc.clear();
  doc.garbageCollect();

  DeserializationError error = deserializeJson(doc, message); 

  if (error) {
    Serial.print("deserializeJson() failed with code ");
    Serial.println(error.c_str());
    return false;
  }

  if (doc.containsKey("state")) {
    if (strcmp(doc["state"], CONFIG_MQTT_PAYLOAD_ON) == 0) {
      stateOn = true;
    }
    else if (strcmp(doc["state"], CONFIG_MQTT_PAYLOAD_OFF) == 0) {
      stateOn = false;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (doc.containsKey("flash") ||
       (doc.containsKey("effect") && strcmp(doc["effect"], "flash") == 0)) {

    if (doc.containsKey("flash")) {
      flashLength = (int)doc["flash"] * 1000;
    }
    else {
      flashLength = CONFIG_DEFAULT_FLASH_LENGTH * 1000;
    }

    if (doc.containsKey("brightness")) {
      flashBrightness = doc["brightness"];
    }
    else {
      flashBrightness = brightness;
    }

    if (rgb && doc.containsKey("color")) {
      flashRed = doc["color"]["r"];
      flashGreen = doc["color"]["g"];
      flashBlue = doc["color"]["b"];
    }
    else {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    if (includeWhite && doc.containsKey("white_value")) {
      flashWhite = doc["white_value"];
    }
    else {
      flashWhite = white;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);
    flashWhite = map(flashWhite, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else if (rgb && doc.containsKey("effect") &&
      (strcmp(doc["effect"], "colorfade_slow") == 0 || strcmp(doc["effect"], "colorfade_fast") == 0)) {
    flash = false;
    colorfade = true;
    currentColor = 0;
    if (strcmp(doc["effect"], "colorfade_slow") == 0) {
      transitionTime = CONFIG_COLORFADE_TIME_SLOW;
    }
    else {
      transitionTime = CONFIG_COLORFADE_TIME_FAST;
    }
  }
  else if (colorfade && !doc.containsKey("color") && doc.containsKey("brightness")) {
    // Adjust brightness during colorfade
    // (will be applied when fading to the next color)
    brightness = doc["brightness"];
  }
  else { // No effect
    flash = false;
    colorfade = false;

    if (rgb && doc.containsKey("color")) {
      red = doc["color"]["r"];
      green = doc["color"]["g"];
      blue = doc["color"]["b"];
    }

    if (includeWhite && doc.containsKey("white_value")) {
      white = doc["white_value"];
    }

    if (doc.containsKey("brightness")) {
      brightness = doc["brightness"];
    }

    if (doc.containsKey("transition")) {
      transitionTime = doc["transition"];
    }
    else {
      transitionTime = CONFIG_DEFAULT_TRANSITION_TIME;
    }
  }

  return true;
}
void sendState() {
  doc.clear();
  doc.garbageCollect();

  doc["state"] = (stateOn) ? CONFIG_MQTT_PAYLOAD_ON : CONFIG_MQTT_PAYLOAD_OFF;
  if (rgb) {
    JsonObject color = doc.createNestedObject("color");
    color["r"] = red;
    color["g"] = green;
    color["b"] = blue;
  }

  doc["brightness"] = brightness;

  if (includeWhite) {
    doc["white_value"] = white;
  }

  if (rgb && colorfade) {
    if (transitionTime == CONFIG_COLORFADE_TIME_SLOW) {
      doc["effect"] = "colorfade_slow";
    }
    else {
      doc["effect"] = "colorfade_fast";
    }
  }
  else {
    doc["effect"] = "null";
  }

  size_t size = measureJson(doc) + 1;

  char buffer[size];
  serializeJson(doc, buffer, size);

  client.publish(CONFIG_MQTT_TOPIC_STATE, buffer, true);
}

void sendDiscovery() {
  doc.clear();
  doc.garbageCollect();
  doc["name"] = CONFIG_MQTT_NAME;
  doc["uniq_id"] = CONFIG_MQTT_CLIENT_ID;
  doc["~"] = CONFIG_MQTT_BASE_TOPIC;
  doc["stat_t"] = CONFIG_MQTT_TOPIC_STATE;
  doc["cmd_t"] = CONFIG_MQTT_TOPIC_SET;
  doc["stat_val_tpl"] = CONFIG_MQTT_STATE_VALUE_TEMPLATE;
  doc["bri_cmd_t"] = CONFIG_MQTT_TOPIC_BRIGHTNESS_SET;
  doc["bri_val_tpl"] = CONFIG_MQTT_BRIGHTNESS_VALUE_TEMPLATE;
  doc["rgb_cmd_t"] = CONFIG_MQTT_TOPIC_RGB_SET;
  doc["rgb_val_tpl"] = CONFIG_MQTT_RGB_VALUE_TEMPLATE;
  doc["optimistic"] = false;
  JsonArray effects = doc.createNestedArray("effect_list");
  effects.add("colorfade_slow");
  effects.add("colorfade_fast");
  effects.add("flash");
  doc["payload_on"] = CONFIG_MQTT_PAYLOAD_ON;
  doc["payload_off"] = CONFIG_MQTT_PAYLOAD_OFF;
  doc["qos"] = 0;

  size_t size = measureJson(doc) + 1;
  Serial.print("size of discovery msg: ");
  Serial.println(size);

  char buffer[size];
  serializeJson(doc, buffer, size);

  bool res = client.publish(CONFIG_MQTT_TOPIC_DISCOVERY, buffer, true);
  if (res) {
    Serial.println("successfully published discovery msg");
  } else {
    Serial.println("discovery msg was not successful.");
  }
}

  /*
  SAMPLE PAYLOAD (BRIGHTNESS):
    {
      "brightness": 120,
      "flash": 2,
      "transition": 5,
      "state": "ON"
    }

  SAMPLE PAYLOAD (RGBW):
    {
      "brightness": 120,
      "color": {
        "r": 255,
        "g": 100,
        "b": 100
      },
      "white_value": 255,
      "flash": 2,
      "transition": 5,
      "state": "ON",
      "effect": "colorfade_fast"
    }
  */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (unsigned int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);
  
  // home assistant sends not valid json for ON and OFF
  if (strcmp(topic, CONFIG_MQTT_TOPIC_SET) == 0) {
    if (strcmp(message, CONFIG_MQTT_PAYLOAD_ON) == 0) {
      stateOn = true;
    } else if (strcmp(message, CONFIG_MQTT_PAYLOAD_OFF) == 0) {
      stateOn = false;
    }
  } else if (strcmp(topic, CONFIG_MQTT_TOPIC_BRIGHTNESS_SET) == 0) {
    brightness = atoi(message);
    stateOn = true;
    Serial.println(brightness);
  } else if (strcmp(topic, CONFIG_MQTT_TOPIC_RGB_SET) == 0) {
    red = atoi(message);
    green = atoi(&strchr(message, ',')[1]);
    blue = atoi(&strrchr(message, ',')[1]);
    Serial.println(red);
    Serial.println(green);
    Serial.println(blue);
    
  } else if (!processJson(message)) {
    return;
  }

  if (stateOn) {
    // Update lights
    realRed = map(red, 0, 255, 0, brightness);
    realGreen = map(green, 0, 255, 0, brightness);
    realBlue = map(blue, 0, 255, 0, brightness);
    realWhite = map(white, 0, 255, 0, brightness);
  }
  else {
    realRed = 0;
    realGreen = 0;
    realBlue = 0;
    realWhite = 0;
  }

  startFade = true;
  inFade = false; // Kill the current fade

  sendState();
}




void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CONFIG_MQTT_CLIENT_ID, CONFIG_MQTT_USER, CONFIG_MQTT_PASS)) {
      Serial.println("connected");
      client.setBufferSize(1024);
      client.subscribe(CONFIG_MQTT_TOPIC_SET);
      client.subscribe(CONFIG_MQTT_TOPIC_BRIGHTNESS_SET);
      client.subscribe(CONFIG_MQTT_TOPIC_RGB_SET);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setColor(int inR, int inG, int inB, int inW) {
  if (CONFIG_INVERT_LED_LOGIC) {
    inR = (255 - inR);
    inG = (255 - inG);
    inB = (255 - inB);
    inW = (255 - inW);
  }

  if (rgb) {
    analogWrite(CONFIG_PIN_RED, inR);
    analogWrite(CONFIG_PIN_GREEN, inG);
    analogWrite(CONFIG_PIN_BLUE, inB);
  }

  if (includeWhite) {
    analogWrite(CONFIG_PIN_WHITE, inW);
  }

  if (CONFIG_DEBUG) {
    /*
    Serial.print("Setting LEDs: {");
    if (rgb) {
      Serial.print("r: ");
      Serial.print(inR);
      Serial.print(" , g: ");
      Serial.print(inG);
      Serial.print(" , b: ");
      Serial.print(inB);
    }

    if (includeWhite) {
      if (rgb) {
        Serial.print(", ");
      }
      Serial.print("w: ");
      Serial.print(inW);
    }

    Serial.println("}");
    */
  }
}



// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS
*
* The program works like this:
* Imagine a crossfade that moves the red LED from 0-10,
*   the green from 0-5, and the blue from 10 to 7, in
*   ten steps.
*   We'd want to count the 10 steps and increase or
*   decrease color values in evenly stepped increments.
*   Imagine a + indicates raising a value by 1, and a -
*   equals lowering it. Our 10 step fade would look like:
*
*   1 2 3 4 5 6 7 8 9 10
* R + + + + + + + + + +
* G   +   +   +   +   +
* B     -     -     -
*
* The red rises from 0 to 10 in ten steps, the green from
* 0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.
*
* In the real program, the color percentages are converted to
* 0-255 values, and there are 1020 steps (255*4).
*
* To figure out how big a step there should be between one up- or
* down-tick of one of the LED values, we call calculateStep(),
* which calculates the absolute gap between the start and end values,
* and then divides that gap by 1020 to determine the size of the step
* between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue) {
    int step = endValue - prevValue; // What's the overall gap?
    if (step) {                      // If its non-zero,
        step = 1020/step;            //   divide by 1020
    }

    return step;
}

/* The next function is calculateVal. When the loop value, i,
*  reaches the step size appropriate for one of the
*  colors, it increases or decreases the value of that color by 1.
*  (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
    if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
        if (step > 0) {              //   increment the value if step is positive...
            val += 1;
        }
        else if (step < 0) {         //   ...or decrement it if step is negative
            val -= 1;
        }
    }

    // Defensive driving: make sure val stays in the range 0-255
    if (val > 255) {
        val = 255;
    }
    else if (val < 0) {
        val = 0;
    }

    return val;
}

void setup() {
  if (rgb) {
    pinMode(CONFIG_PIN_RED, OUTPUT);
    pinMode(CONFIG_PIN_GREEN, OUTPUT);
    pinMode(CONFIG_PIN_BLUE, OUTPUT);
  }
  if (includeWhite) {
    pinMode(CONFIG_PIN_WHITE, OUTPUT);
  }

  // Set the LED_BUILTIN based on the CONFIG_LED_BUILTIN_MODE
  switch (CONFIG_LED_BUILTIN_MODE) {
    case 0:
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case 1:
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    default: // Other options (like -1) are ignored.
      break;
  }

  analogWriteRange(255);

  if (CONFIG_DEBUG) {
    Serial.begin(115200);
  }

  setup_wifi();
  client.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  client.setCallback(callback);
  while (!client.connected()) {
    reconnect();
    delay(500);
  }
  sendDiscovery();
  setColor(realRed, realGreen, realBlue, realWhite);
  sendState();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue, flashWhite);
      }
      else {
        setColor(0, 0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue, realWhite);
      }
    }
    else {
      flash = false;
      setColor(realRed, realGreen, realBlue, realWhite);
    }
  }
  else if (rgb && colorfade && !inFade) {
    realRed = map(colors[currentColor][0], 0, 255, 0, brightness);
    realGreen = map(colors[currentColor][1], 0, 255, 0, brightness);
    realBlue = map(colors[currentColor][2], 0, 255, 0, brightness);
    realWhite = map(colors[currentColor][3], 0, 255, 0, brightness);
    currentColor = (currentColor + 1) % numColors;
    startFade = true;
  }

  if (startFade) {
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      setColor(realRed, realGreen, realBlue, realWhite);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;
      whtVal = realWhite;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);
      stepW = calculateStep(whtVal, realWhite);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);
        whtVal = calculateVal(stepW, whtVal, loopCount);

        setColor(redVal, grnVal, bluVal, whtVal); // Write current values to LED pins

        //Serial.print("Loop count: ");
        //Serial.println(loopCount);
        //loopCount++;
      }
      else {
        inFade = false;
      }
    }
  }
}
