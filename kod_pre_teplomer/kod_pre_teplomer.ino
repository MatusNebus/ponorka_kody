#include <OneWire.h>         // Include the OneWire library to communicate with the sensor
#include <DallasTemperature.h> // Include the DallasTemperature library for DS18B20

// Define the pin where the data wire is connected
const int oneWireBus = 11;  // Digital pin 11 (you can use other pins as well)

OneWire oneWire(oneWireBus);              // Create a OneWire object
DallasTemperature sensors(&oneWire);      // Create a DallasTemperature object

void setup() {
  Serial.begin(9600);     // Start serial communication for debugging
  sensors.begin();        // Initialize the sensor
}

void loop() {
  sensors.requestTemperatures();  // Request temperature readings from the DS18B20
  float temperature = sensors.getTempCByIndex(0);  // Get temperature in Celsius

  Serial.print("Temperature: ");
  Serial.print(temperature);  // Print the temperature in Celsius
  Serial.println(" °C");

  delay(1000);  // Wait for 1 second before reading the temperature again
}
