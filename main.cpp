#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <ACS712.h>

// Pin Definitions
#define TEMP_SENSOR_PIN A0  // Analog pin for LM35 temperature sensor
#define MOSFET_PIN 9        // PWM pin for MOSFET control
#define MAX_TEMPERATURE 45  // Maximum temperature threshold in Celsius

// Create sensor instances
Adafruit_INA219 ina219;   // Voltage sensor
ACS712 currentSensor(ACS712_20A, A1); // Current sensor
LiquidCrystal_I2C lcd(0x27, 16, 2);   // LCD display setup

// Variables for sensor data
float voltage = 0.0;
float current = 0.0;
float temperature = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.begin();
  lcd.backlight();
  
  // Initialize INA219 voltage sensor
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  // Set PWM pin as output for MOSFET control
  pinMode(MOSFET_PIN, OUTPUT);
  
  // Display setup status
  lcd.setCursor(0, 0);
  lcd.print("Charger Ready");
}

void loop() {
  // Read sensor values
  voltage = ina219.getBusVoltage_V();
  current = currentSensor.getCurrentAC();
  temperature = readTemperature();

  // Display data on LCD
  lcd.setCursor(0, 0);
  lcd.print("Volt: ");
  lcd.print(voltage);
  lcd.print("V");

  lcd.setCursor(0, 1);
  lcd.print("Curr: ");
  lcd.print(current);
  lcd.print("A");
  
  // Check temperature safety
  if (temperature > MAX_TEMPERATURE) {
    stopCharging();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Overheat!");
    Serial.println("Overheat detected! Charging stopped.");
    delay(1000); // Wait before restarting loop
    return;
  }

  // Execute charging profile
  if (voltage < 4.2) {
    // Constant current phase
    controlPWM(128); // Example PWM duty cycle, adjust as needed
    Serial.println("Charging in CC phase.");
  } else {
    // Constant voltage phase
    controlPWM(64);  // Reduce current in CV phase
    Serial.println("Charging in CV phase.");
  }

  delay(500);  // Delay for data stability
}

// Function to read temperature from LM35
float readTemperature() {
  int analogValue = analogRead(TEMP_SENSOR_PIN);
  float millivolts = (analogValue / 1024.0) * 5000;
  float temperatureC = millivolts / 10;  // 10mV per degree Celsius
  return temperatureC;
}

// Function to control PWM output to MOSFET
void controlPWM(int dutyCycle) {
  analogWrite(MOSFET_PIN, dutyCycle);
}

// Function to stop charging (set PWM to 0)
void stopCharging() {
  analogWrite(MOSFET_PIN, 0);
}
