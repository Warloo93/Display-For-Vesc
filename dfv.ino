#include <FlickerFreePrint.h>
#include <VescUart.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "EEPROMAnything.h"
#include <WiFi.h>
#include "mainscreen.h" // Include mainscreen bitmap data
#include "secondscreen.h" // Include secondscreen bitmap data
#include "ProFontWindows82pt7b.h"
#include "ProFontWindows24pt7b.h"
#include "ProFontWindows18pt7b.h"

// CAN IDs for the two VESCs
#define VESC1_CAN_ID 1
#define VESC2_CAN_ID 2
#define TFT_BL 25

// Pin definitions
#define FUNK_BUTTON_PIN 15    // Funk button (D15)
#define BRAKE_LIGHT_PIN 2     // Brake light (PWM pin for MOSFET) - LEDC new API used

// Custom struct to hold VESC data
struct VescData {
  float tempMosfet;
  float tempMotor;
  float inpVoltage;
  float rpm;
  int32_t tachometerAbs;
  int32_t tachometer;
  float avgInputCurrent;
};

// Variables
float trip = 0.0;
float startup_total_km = 0.0;
float last_total_km_stored = 0.0;
float total_km = 0.0;
float speed = 0.0;
float battery_power = 0.0;

// Brake light variables
bool isBraking = false;
unsigned long lastBlinkTime = 0;
// NOTE: brake_flash_interval will be calculated as half-period (ON or OFF duration)
unsigned long brake_flash_interval = 25; // placeholder, will be recalculated in setup
bool brakeBlinkState = false;
byte base_bright_percent = 50; // Default 50% brightness
byte flash_hz = 20; // Default 20 Hz

// Funk button variables
bool limitActive = false; // Speed limit state (will be forced ON at startup)
unsigned long lastFunkPress = 0;
unsigned long funkPressStart = 0; // Button press start
const unsigned long LIMIT_HOLD_TIME = 5000; // 5 seconds for limit toggle
const unsigned long DEBOUNCE_DELAY = 200;

// Speed and power limit settings
const float MAX_SPEED_LIMIT = 25.0 / 3.6; // 25 km/h = 6.944 m/s

// Wheel diameter and config settings
int wheel_diameter = 200;
int motor_pole_pairs = 30; // Set via web interface
bool is_dual_motor = true;
const int MIN_WHEEL_DIAMETER = 150;
const int MAX_WHEEL_DIAMETER = 700;

// WiFi settings
const char* ssid = "VESC_Config";
const char* password = "12345678";
WiFiServer server(80);

// EEPROM addresses
#define EEPROM_WHEEL_ADDR 0
#define EEPROM_POLE_ADDR 4
#define EEPROM_TOTAL_KM_ADDR 12
#define EEPROM_LIMIT_ADDR 20 // Limit state
#define EEPROM_SCREEN_ADDR 24 // Current screen (optional)
#define EEPROM_BASE_BRIGHT 28
#define EEPROM_FLASH_HZ 29

// Wheel Settings
#define PI 3.14159

// User Settings
#define EEPROM_UPDATE_EACH_KM 0.1

VescUart UART; // VescUart object
VescData vesc1Data;
VescData vesc2Data;
TFT_eSPI tft = TFT_eSPI();

// Screen management
bool currentScreen = false; // false: main screen, true: second screen
bool needRedraw = true; // Flag to redraw screen on switch

// Track max values
float max_speed = 0.0;
float max_power = 0.0;

// FlickerFreePrint objects for main screen
FlickerFreePrint<TFT_eSPI> MainSpeed(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> MainOdo(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> MainTrip(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> MainVoltage(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> MainPower(&tft, TFT_WHITE, TFT_BLACK);

// FlickerFreePrint objects for second screen
FlickerFreePrint<TFT_eSPI> SecondVesc1Temp(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> SecondVesc2Temp(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> SecondMotor1Temp(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> SecondMotor2Temp(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> SecondTopSpeed(&tft, TFT_WHITE, TFT_BLACK);
FlickerFreePrint<TFT_eSPI> SecondMaxPower(&tft, TFT_WHITE, TFT_BLACK);

// Function prototype for applySpeedPowerLimit
void applySpeedPowerLimit(bool enable);

void setup(void) {
  delay(6000);
  // Initialize pins
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  pinMode(FUNK_BUTTON_PIN, INPUT_PULLUP);
  // Note: With new LEDC API we don't need to rely on analogWrite pin-mode for PWM,
  // but keeping pinMode doesn't hurt for safety.
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  // Initialize serial communication
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // UART: TX=17, RX=16
  UART.setSerialPort(&Serial2);
  UART.setDebugPort(&Serial);

  // Initialize TFT
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Display "Connecting to VESC" message
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Connecting to VESC", 160, 240);
  
  // Initialize EEPROM
  EEPROM.begin(104); // Increased to 104 bytes

  // Load from EEPROM with validation
  wheel_diameter = EEPROM.readInt(EEPROM_WHEEL_ADDR);
  if (wheel_diameter < MIN_WHEEL_DIAMETER || wheel_diameter > MAX_WHEEL_DIAMETER) {
    wheel_diameter = 200;
    EEPROM.writeInt(EEPROM_WHEEL_ADDR, wheel_diameter);
  }
  motor_pole_pairs = EEPROM.readInt(EEPROM_POLE_ADDR);
  if (motor_pole_pairs < 1 || motor_pole_pairs > 50) {
    motor_pole_pairs = 30;
    EEPROM.writeInt(EEPROM_POLE_ADDR, motor_pole_pairs);
  }
  startup_total_km = EEPROM.readFloat(EEPROM_TOTAL_KM_ADDR);
  if (isnan(startup_total_km) || startup_total_km < 0) {
    startup_total_km = 0;
    EEPROM.writeFloat(EEPROM_TOTAL_KM_ADDR, startup_total_km);
  }

  // Read base brightness and flash hz from EEPROM (with validation)
  base_bright_percent = EEPROM.readByte(EEPROM_BASE_BRIGHT);
  if (base_bright_percent > 100) {
    base_bright_percent = 50;
    EEPROM.writeByte(EEPROM_BASE_BRIGHT, base_bright_percent);
  }
  flash_hz = EEPROM.readByte(EEPROM_FLASH_HZ);
  if (flash_hz < 1 || flash_hz > 50) {
    flash_hz = 20;
    EEPROM.writeByte(EEPROM_FLASH_HZ, flash_hz);
  }

  // Load screen selection
  byte screenByte = EEPROM.readByte(EEPROM_SCREEN_ADDR);
  currentScreen = (screenByte == 1);

  last_total_km_stored = startup_total_km;

  // FORCE limit to be active at every startup (user requested)
  limitActive = true;
  EEPROM.writeByte(EEPROM_LIMIT_ADDR, 1);

  EEPROM.commit();

  // Calculate initial brake flash interval (half period in ms)
  brake_flash_interval = 1000UL / ((unsigned long)flash_hz * 2UL);

  // Initialize LEDC using new API: ledcAttach(pin, freq, resolution)
  ledcAttach(BRAKE_LIGHT_PIN, 5000, 8);

  // Apply initial brightness using ledcWrite (new API)
  int bright = map(base_bright_percent, 0, 100, 0, 255);
  ledcWrite(BRAKE_LIGHT_PIN, bright);

  // Initialize speed/power limit (limitActive is forced true)
  if (limitActive) {
    applySpeedPowerLimit(true);
  }

  // Start WiFi AP
  WiFi.softAP(ssid, password);
  server.begin();

  // Wait for VESC connection (min 2s, max 15s)
  unsigned long startTime = millis();
  bool vescConnected = false;
  while (millis() - startTime < 15000) {
    if (UART.getVescValues()) {
      vescConnected = true;
      break;
    }
    delay(100);
  }
  if (!vescConnected && millis() - startTime < 2000) {
    delay(2000 - (millis() - startTime));
  }
  
  // Clear the "Connecting to VESC" message
  tft.fillScreen(TFT_BLACK);

  Serial.println("Setup completed");
  Serial.print("Initial base_bright_percent: ");
  Serial.println(base_bright_percent);
  Serial.print("Initial flash_hz: ");
  Serial.println(flash_hz);
  Serial.print("Brake half-period (ms): ");
  Serial.println(brake_flash_interval);
}

// Apply speed and power limit
void applySpeedPowerLimit(bool enable) {
  bool store = false;
  bool forward_can = is_dual_motor;
  bool divide_by_controllers = is_dual_motor;
  float current_min_rel = 1.0;
  float current_max_rel = 1.0;
  float duty_min = 0.005;
  float duty_max = 0.95;

  float watt_max_unlimited = 1500000.0;
  float watt_min_unlimited = -1500000.0;

  if (enable) {
    float speed_max = MAX_SPEED_LIMIT;
    float speed_max_reverse = -MAX_SPEED_LIMIT;
    
    UART.setLocalProfile(store, forward_can, divide_by_controllers, current_min_rel, current_max_rel,
                         speed_max_reverse, speed_max, duty_min, duty_max, watt_min_unlimited, watt_max_unlimited);
    Serial.println("Limit applied: 25km/h");
  } else {
    float speed_max = 27.78;
    float speed_max_reverse = -27.78;

    UART.setLocalProfile(store, forward_can, divide_by_controllers, current_min_rel, current_max_rel,
                         speed_max_reverse, speed_max, duty_min, duty_max, watt_min_unlimited, watt_max_unlimited);
    Serial.println("Limit removed");
  }
}

void loop() {
  // Track last limit state to detect changes
  static bool lastLimitActive = limitActive;

  // Funk button check
  if (digitalRead(FUNK_BUTTON_PIN) == LOW) {
    if (funkPressStart == 0) {
      funkPressStart = millis();
    } else if (millis() - funkPressStart >= LIMIT_HOLD_TIME && (millis() - lastFunkPress > DEBOUNCE_DELAY)) {
      limitActive = !limitActive;
      lastFunkPress = millis();
      
      applySpeedPowerLimit(limitActive);

      EEPROM.writeByte(EEPROM_LIMIT_ADDR, limitActive ? 1 : 0);
      EEPROM.commit();
      Serial.println("Limit toggled, EEPROM saved");
      funkPressStart = 0;
    }
  } else {
    // Button released
    if (funkPressStart != 0 && (millis() - funkPressStart < LIMIT_HOLD_TIME) && (millis() - lastFunkPress > DEBOUNCE_DELAY)) {
      // Short press: toggle screen
      currentScreen = !currentScreen;
      needRedraw = true;
      EEPROM.writeByte(EEPROM_SCREEN_ADDR, currentScreen ? 1 : 0);
      EEPROM.commit();
      lastFunkPress = millis();
      Serial.println("Screen toggled, EEPROM saved");
    }
    funkPressStart = 0;
  }

  // Check if limitActive changed to trigger redraw
  if (lastLimitActive != limitActive) {
    needRedraw = true;
    lastLimitActive = limitActive;
  }

  // Fetch VESC data
  if (UART.getVescValues()) {
    vesc1Data.tempMosfet = UART.data.tempMosfet;
    vesc1Data.tempMotor = UART.data.tempMotor;
    vesc1Data.inpVoltage = UART.data.inpVoltage;
    vesc1Data.rpm = UART.data.rpm;
    vesc1Data.tachometerAbs = UART.data.tachometerAbs;
    vesc1Data.tachometer = UART.data.tachometer;
    vesc1Data.avgInputCurrent = UART.data.avgInputCurrent;
  }

  if (is_dual_motor && UART.getVescValues(VESC2_CAN_ID)) {
    vesc2Data.tempMosfet = UART.data.tempMosfet;
    vesc2Data.tempMotor = UART.data.tempMotor;
    vesc2Data.avgInputCurrent = UART.data.avgInputCurrent;
  } else {
    vesc2Data.avgInputCurrent = 0;
    vesc2Data.tempMosfet = 0; // Set to 0 if not dual
    vesc2Data.tempMotor = 0;
  }

  // Calculate speed and distance
  float wheel_circumference = PI * wheel_diameter / 1000.0;
  float rpm_to_speed = (wheel_circumference * 60) / 1000.0;
  speed = (vesc1Data.rpm / motor_pole_pairs) * rpm_to_speed;

  // Distance calculation using tachometer
  trip = ((float)vesc1Data.tachometer / (motor_pole_pairs * 6.0)) * (wheel_circumference / 1000.0);
  total_km = startup_total_km + trip;

  battery_power = (vesc1Data.avgInputCurrent + vesc2Data.avgInputCurrent) * vesc1Data.inpVoltage;
  isBraking = (battery_power < -50);

  // Update max values (only positive power)
  if (speed > max_speed) max_speed = speed;
  if (battery_power > max_power) max_power = battery_power;

  if (total_km - last_total_km_stored >= EEPROM_UPDATE_EACH_KM) {
    last_total_km_stored = total_km;
    EEPROM.writeFloat(EEPROM_TOTAL_KM_ADDR, total_km);
    EEPROM.commit();
    Serial.println("Odometer saved to EEPROM");
  }

  // LED control using new LEDC API
  if (isBraking) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= brake_flash_interval) {
      brakeBlinkState = !brakeBlinkState;
      ledcWrite(BRAKE_LIGHT_PIN, brakeBlinkState ? 255 : 0); // Full brightness (255) during flash
      lastBlinkTime = now;
      Serial.print("Braking: PWM set to ");
      Serial.println(brakeBlinkState ? 255 : 0);
    }
  } else {
    int bright = map(base_bright_percent, 0, 100, 0, 255);
    ledcWrite(BRAKE_LIGHT_PIN, bright); // Apply base brightness
    Serial.print("Non-braking: PWM set to ");
    Serial.println(bright);
  }

  // Display update
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 100) {
    // Debug print
    Serial.print("Tachometer: ");
    Serial.print(vesc1Data.tachometer);
    Serial.print(", Trip: ");
    Serial.print(trip, 2);
    Serial.print(", Total KM: ");
    Serial.print(total_km, 1);
    Serial.print(", Wheel Circumference: ");
    Serial.println(wheel_circumference, 4);

    char fmt[20]; // Buffer for string formatting

    if (needRedraw) {
      tft.fillScreen(TFT_BLACK);
      if (!currentScreen) {
        // Main screen bitmap
        tft.drawBitmap(5, 162, image_Layer_15_bits, 308, 287, TFT_WHITE);
        // Draw red rectangle with "NO LIMIT" text if limit is not active
        if (!limitActive) {
          tft.fillRect(20, 0, 280, 65, TFT_RED); // Red rectangle at top
          tft.setTextColor(TFT_BLACK, TFT_RED); // Black text on red background
          tft.setFreeFont(&FreeSansBold24pt7b); // Large bold font
          tft.setTextSize(1);
          tft.setTextDatum(MC_DATUM); // Middle center datum
          tft.drawString("NO LIMIT", 160, 32); // Center of rectangle
          tft.setTextDatum(TL_DATUM); // Reset to top-left
        }
      } else {
        // Second screen bitmap
        tft.drawBitmap(16, 20, image_Layer_24_bits, 287, 369, TFT_WHITE);
      }
      needRedraw = false;
    }

    if (!currentScreen) {
      // Main screen updates
      // Speed
      tft.setTextSize(1);
      tft.setFreeFont(&ProFontWindows82pt7b);
      tft.setCursor(10, 180);
      dtostrf(max(speed, 0.0f), 3, 0, fmt); // Use standard max function
      MainSpeed.print(fmt);

      // Odometer (left bottom)
      tft.setTextSize(1);
      tft.setFreeFont(&FreeMonoBoldOblique12pt7b);
      tft.setCursor(3, 475);
      dtostrf(total_km, 6, 1, fmt);
      MainOdo.print(fmt);

      // Trip (right bottom)
      tft.setCursor(243, 475);
      dtostrf(trip, 5, 1, fmt);
      MainTrip.print(fmt);

      // Voltage
      tft.setFreeFont(&ProFontWindows24pt7b);
      tft.setCursor(64, 367);
      dtostrf(vesc1Data.inpVoltage, 4, 1, fmt);
      MainVoltage.print(fmt);

      // Power
      tft.setCursor(40, 305);
      dtostrf(battery_power, 5, 0, fmt);
      MainPower.print(fmt);
    } else {
      // Second screen updates
      tft.setTextSize(1);
      tft.setFreeFont(&ProFontWindows18pt7b);

      // First VESC temp
      tft.setCursor(230, 83);
      dtostrf(vesc1Data.tempMosfet, 3, 0, fmt);
      SecondVesc1Temp.print(fmt);

      // Rear VESC temp
      tft.setCursor(231, 121);
      dtostrf(vesc2Data.tempMosfet, 3, 0, fmt);
      SecondVesc2Temp.print(fmt);

      // First motor temp
      tft.setCursor(231, 212);
      dtostrf(vesc1Data.tempMotor, 3, 0, fmt);
      SecondMotor1Temp.print(fmt);

      // Rear motor temp
      tft.setCursor(231, 249);
      dtostrf(vesc2Data.tempMotor, 3, 0, fmt);
      SecondMotor2Temp.print(fmt);

      // Top speed
      tft.setCursor(231, 342);
      dtostrf(max_speed, 3, 0, fmt);
      SecondTopSpeed.print(fmt);

      // Max power
      tft.setCursor(210, 380);
      dtostrf(max_power, 4, 0, fmt);
      SecondMaxPower.print(fmt);
    }

    lastDisplayUpdate = millis();
  }

  // WiFi configuration
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck >= 500) {
    WiFiClient client = server.available();
    if (client) {
      String header = "";
      unsigned long timeout = millis() + 2000;
      while (client.connected() && millis() < timeout) {
        if (client.available()) {
          char c = client.read();
          header += c;
          if (header.endsWith("\r\n\r\n")) {
            break;
          }
        }
      }

      Serial.print("Received header: ");
      Serial.println(header); // Debug header

      if (header.indexOf("POST /set HTTP") != -1) {
        // Parse Content-Length
        int contentLengthPos = header.indexOf("Content-Length: ");
        int contentLength = 0;
        if (contentLengthPos != -1) {
          int endLine = header.indexOf("\r\n", contentLengthPos);
          String lengthStr = header.substring(contentLengthPos + 16, endLine);
          contentLength = lengthStr.toInt();
        }

        // Read body
        String body = "";
        timeout = millis() + 2000;
        while (body.length() < contentLength && client.connected() && millis() < timeout) {
          if (client.available()) {
            body += (char)client.read();
          }
        }

        Serial.print("Received body: ");
        Serial.println(body); // Debug body

        // Parse parameters from body
        int wheelPos = body.indexOf("wheel=") + 6;
        int polePos = body.indexOf("&pole=") + 6;
        int basePos = body.indexOf("&base_bright=") + 13;
        int flashPos = body.indexOf("&flash_hz=") + 10;

        String wheelStr = body.substring(wheelPos, body.indexOf("&", wheelPos));
        String poleStr = body.substring(polePos, body.indexOf("&", polePos));
        String baseStr = body.substring(basePos, body.indexOf("&", basePos));
        String flashStr = body.substring(flashPos);

        int new_wheel = wheelStr.toInt();
        int new_pole = poleStr.toInt();
        byte new_base = baseStr.toInt();
        byte new_flash = flashStr.toInt();

        // Debug parsed values
        Serial.print("Parsed - Wheel: ");
        Serial.print(new_wheel);
        Serial.print(", Pole: ");
        Serial.print(new_pole);
        Serial.print(", Base Bright: ");
        Serial.print(new_base);
        Serial.print(", Flash Hz: ");
        Serial.println(new_flash);

        // Validate and update parameters
        bool settingsChanged = false;
        if (new_wheel >= MIN_WHEEL_DIAMETER && new_wheel <= MAX_WHEEL_DIAMETER) {
          wheel_diameter = new_wheel;
          EEPROM.writeInt(EEPROM_WHEEL_ADDR, wheel_diameter);
          settingsChanged = true;
        }
        if (new_pole > 0 && new_pole <= 50) {
          motor_pole_pairs = new_pole;
          EEPROM.writeInt(EEPROM_POLE_ADDR, motor_pole_pairs);
          settingsChanged = true;
        }
        if (new_base >= 0 && new_base <= 100) {
          base_bright_percent = new_base;
          EEPROM.writeByte(EEPROM_BASE_BRIGHT, base_bright_percent);
          settingsChanged = true;
          if (!isBraking) {
            int brightNow = map(base_bright_percent, 0, 100, 0, 255);
            ledcWrite(BRAKE_LIGHT_PIN, brightNow);
            Serial.print("Applied new base brightness: ");
            Serial.println(brightNow);
          }
        }
        if (new_flash >= 1 && new_flash <= 50) {
          flash_hz = new_flash;
          brake_flash_interval = 1000UL / ((unsigned long)flash_hz * 2UL);
          EEPROM.writeByte(EEPROM_FLASH_HZ, flash_hz);
          settingsChanged = true;
          Serial.print("Applied new flash_hz: ");
          Serial.print(flash_hz);
          Serial.print(" -> half-period (ms): ");
          Serial.println(brake_flash_interval);
        }

        if (settingsChanged) {
          EEPROM.commit();
          Serial.println("Settings saved to EEPROM");
        }

        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/plain");
        client.println("Connection: close");
        client.println();
        client.println("Saved!");
        client.stop();
      } else if (header.indexOf("GET /reset_odo") != -1) {
        startup_total_km = 0;
        total_km = 0;
        last_total_km_stored = 0;
        trip = 0;
        EEPROM.writeFloat(EEPROM_TOTAL_KM_ADDR, 0);
        EEPROM.commit();
        needRedraw = true;
        Serial.println("Odometer reset, EEPROM saved");
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println("Connection: close");
        client.println();
        client.println("<html><body><h1>Odometer reset, refreshing...</h1></body></html>");
        client.stop();
      } else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println("Connection: close");
        client.println();
        client.println(
          "<!DOCTYPE html>"
          "<html>"
          "<head>"
          "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
          "<title>VESC Config</title>"
          "<style>"
          "body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif, 'Apple Color Emoji', 'Segoe UI Emoji', 'Segoe UI Symbol'; background-color: #121212; color: #e0e0e0; margin: 0; padding: 20px; }"
          ".container { max-width: 500px; margin: auto; background-color: #1e1e1e; padding: 30px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0, 0, 0, 0.3); }"
          "h1 { text-align: center; color: #fff; margin-bottom: 25px; font-weight: 600; }"
          "form { display: flex; flex-direction: column; gap: 20px; }"
          ".form-group { display: flex; flex-direction: column; }"
          "label { font-weight: bold; margin-bottom: 5px; color: #c0c0c0; }"
          "input[type=\"number\"] { width: 100%; padding: 12px; box-sizing: border-box; border: 1px solid #333; border-radius: 8px; background-color: #2a2a2a; color: #fff; font-size: 16px; }"
          "input[type=\"submit\"] { background-color: #007aff; color: #fff; padding: 14px; border: none; border-radius: 8px; cursor: pointer; font-size: 18px; font-weight: 600; transition: background-color 0.3s ease; }"
          "input[type=\"submit\"]:hover { background-color: #005bb5; }"
          ".info { margin-top: 30px; padding-top: 25px; border-top: 1px solid #333; }"
          ".info p { margin: 8px 0; color: #b0b0b0; }"
          "</style>"
          "<script>"
          "function saveConfig() {"
          "var wheel = document.getElementById('wheel').value;"
          "var pole = document.getElementById('pole').value;"
          "var base_bright = document.getElementById('base_bright').value;"
          "var flash_hz = document.getElementById('flash_hz').value;"
          "var xhr = new XMLHttpRequest();"
          "xhr.open('POST', '/set', true);"
          "xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded');"
          "xhr.onreadystatechange = function() {"
          "if (this.readyState === XMLHttpRequest.DONE && this.status === 200) {"
          "document.getElementById('message').innerHTML = 'Saved!';"
          "document.getElementById('curr_wheel').innerHTML = wheel + ' mm';"
          "document.getElementById('curr_pole').innerHTML = pole;"
          "document.getElementById('curr_base').innerHTML = base_bright + '%';"
          "document.getElementById('curr_flash').innerHTML = flash_hz + ' Hz';"
          "}"
          "};"
          "xhr.send('wheel=' + wheel + '&pole=' + pole + '&base_bright=' + base_bright + '&flash_hz=' + flash_hz);"
          "return false;"
          "}"
          "</script>"
          "</head>"
          "<body>"
          "<div class=\"container\">"
          "<h1>VESC Configurator</h1>"
          "<form onsubmit=\"return saveConfig();\">"
          "<div class=\"form-group\">"
          "<label for=\"wheel\">Wheel Diameter (mm, 150-700):</label>"
          "<input type=\"number\" id=\"wheel\" name=\"wheel\" value=\"" + String(wheel_diameter) + "\" min=\"150\" max=\"700\">"
          "</div>"
          "<div class=\"form-group\">"
          "<label for=\"pole\">Motor Pole Pairs (1-50):</label>"
          "<input type=\"number\" id=\"pole\" name=\"pole\" value=\"" + String(motor_pole_pairs) + "\" min=\"1\" max=\"50\">"
          "</div>"
          "<div class=\"form-group\">"
          "<label for=\"base_bright\">Base Brightness (% 0-100):</label>"
          "<input type=\"number\" id=\"base_bright\" name=\"base_bright\" value=\"" + String(base_bright_percent) + "\" min=\"0\" max=\"100\">"
          "</div>"
          "<div class=\"form-group\">"
          "<label for=\"flash_hz\">Brake Flash Frequency (Hz 1-50):</label>"
          "<input type=\"number\" id=\"flash_hz\" name=\"flash_hz\" value=\"" + String(flash_hz) + "\" min=\"1\" max=\"50\">"
          "</div>"
          "<input type=\"submit\" value=\"Save\">"
          "</form>"
          "<p id=\"message\"></p>"
          "<form action=\"/reset_odo\" method=\"get\">"
          "<input type=\"submit\" value=\"Reset Odometer\">"
          "</form>"
          "<div class=\"info\">"
          "<p>Current Wheel Diameter: <b id=\"curr_wheel\">" + String(wheel_diameter) + " mm</b></p>"
          "<p>Current Motor Pole Pairs: <b id=\"curr_pole\">" + String(motor_pole_pairs) + "</b></p>"
          "<p>Current Base Brightness: <b id=\"curr_base\">" + String(base_bright_percent) + "%</b></p>"
          "<p>Current Brake Flash Frequency: <b id=\"curr_flash\">" + String(flash_hz) + " Hz</b></p>"
          "<p>Current Odometer: <b id=\"curr_odo\">" + String(total_km, 1) + " km</b></p>"
          "</div>"
          "</div>"
          "</body>"
          "</html>"
        );
        client.stop();
      }
      lastWiFiCheck = millis();
    }
  }
}