Credits: Based on the original project by [Gh0513d/SVD](https://github.com/Gh0513d/SVD)

Display For VESC
This project is a custom dashboard and configuration tool for personal electric vehicles (PEVs) that use VESC®-based motor controllers. Built on an ESP32 microcontroller with a TFT screen, it provides real-time ride metrics, a brake light controller, and a Wi-Fi-based configuration interface.

🚀 Features
Real-Time Ride Metrics: Displays essential data from your VESC(s) on a connected TFT screen, including:

Current speed (km/h).

Total odometer (km).

Trip distance (km).

Input voltage.

Current power output (Watts).

VESC and motor temperatures.

Max speed and max power.

Dual VESC Support: Automatically detects and displays data for a dual-motor setup.

Brake Light Control: Manages a brake light with a configurable base brightness and a flashing function that activates when braking is detected (when power is negative).

Speed & Power Limiting: A hardware button (FUNK_BUTTON_PIN) can be held to toggle a 25 km/h speed limit. The limit is automatically activated every time the device powers on.

Wi-Fi Configuration Portal: No need to re-flash the code for every change! Connect to the VESC_Config Wi-Fi network to access a simple web page where you can:

Adjust the wheel diameter (in mm) for accurate speed and distance calculations.

Set the motor pole pairs to calibrate RPM.

Configure the base brightness and flashing frequency of the brake light.

Reset the odometer.

Screen Toggling: A short press of the hardware button (FUNK_BUTTON_PIN) cycles between the main dashboard and a secondary screen with detailed metrics like temperatures and max values.

EEPROM Persistence: All settings (odometer, wheel diameter, pole pairs, brake light settings, and speed limit state) are saved to the ESP32's internal EEPROM, so they are remembered after a power cycle.

🛠️ Hardware Requirements
Microcontroller: ESP32 board (tested with ESP32 Dev Module)

Display: ILI9488 3.5" display

Connectivity: Connection to VESC via UART (Serial2 on ESP32, pins 16 and 17).

Buttons: One physical push button for screen/limit control.

Brake Light: A PWM-controlled brake light (e.g., an LED connected via a MOSFET).

⚙️ Installation & Setup
Copy Libraries:

Download the project files from GitHub.

You will find a libraries folder within the project, which contains all the necessary libraries.

Copy all the folders inside this libraries directory to your Arduino IDE's libraries folder. This is typically located at C:\Users\[Your_Username]\Documents\Arduino\libraries.

Hardware Connections:

Configure Display Pinout: Before uploading, you must set the pinout for your specific display in the User_Setup.h file within the TFT_eSPI library folder. This ensures the microcontroller knows how to communicate with the screen.

Connect the display to the ESP32 via SPI. The backlight is on pin 25.

Connect the ESP32's RX2 (pin 16) and TX2 (pin 17) pins to the VESC's TX and RX pins, respectively.

Connect a button from pin 15 (FUNK_BUTTON_PIN) to GND. This uses the ESP32's internal pull-up resistor.

Connect your brake light driver (e.g., MOSFET gate) to pin 2 (BRAKE_LIGHT_PIN).

Upload the Code:

Open the .ino file in the Arduino IDE.

Ensure your board is set to "ESP32 Dev Module" and the correct COM port is selected.

Click "Upload."

📖 Usage
Dashboard Operation
Main Screen: Displays live speed, voltage, power, trip distance, and total odometer.

Secondary Screen: A short press of the Funk Button switches to this screen, which shows motor and VESC temperatures, max speed, and max power.

Speed Limit: Hold the Funk Button for 5 seconds to toggle the 25 km/h speed limit on or off. The main screen will display a red "NO LIMIT" bar when the limit is off. The limit will always be ON at startup.

Wi-Fi Configuration
Power on your board.

On your phone or computer, connect to the Wi-Fi network named VESC_Config with the password 12345678.

Navigate to http://192.168.4.1 in your web browser.

Use the web interface to input your specific settings (wheel diameter, motor pole pairs, etc.).

Click "Save" to apply the changes. They will be saved to EEPROM and take effect immediately.

<img src="https://github.com/Warloo93/Display-For-Vesc/blob/main/IMG_20250826_223225.jpg?raw=true" width="50%"></img> 
<img src="https://github.com/Warloo93/Display-For-Vesc/blob/main/IMG_20250826_223230.jpg?raw=true" width="50%"></img> 

