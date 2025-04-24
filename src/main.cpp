#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

//LM35 
#define ADC_VREF_mV    5000.0 // in millivolt
#define ADC_RESOLUTION 1024.0
#define pin3       A2

// DHT
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN 2
#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;


#define pin1 A0
#define pin2 A1

// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
 
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
 
// Float for Reference Voltage
float ref_voltage = 5.0;
 
// Integer for ADC value
int adc_value = 0;

float getTemprature(uint8_t PIN_LM35){
  // get the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in Celsius
  float tempC = milliVolt / 10;
  // convert the Celsius to Fahrenheit
  //float tempF = tempC * 9 / 5 + 32;

  return tempC;
}
 

float getCurrent(uint8_t pin)
{
  int adc = analogRead(pin);
  float current = -((adc*5/1023.0)-2.5)/0.185;
  return current;
}

float getVoltage(uint8_t pin )
{
  // Read the Analog Input
  adc_value = analogRead(pin);
  // Determine voltage at ADC input
  adc_voltage  = (adc_value * ref_voltage) / 1024.0;
  // Calculate voltage at divider input
  in_voltage = adc_voltage*(R1+R2)/R2;
  if (in_voltage < 0){
    in_voltage = 0;
  }
  // Print results to Serial Monitor to 2 decimal places
  return in_voltage;
}


void setup()
{
  Serial.begin(9600);
  pinMode(4, OUTPUT);
  dht.begin();
  lcd.init();
  lcd.backlight();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  // dht.humidity().getSensor(&sensor);

  delayMS = sensor.min_delay / 1000;
}


void loop()
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));

  float current = getCurrent( pin1);
  float voltage = getVoltage( pin2);

    // Print a message to the LCD.
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("V:");
  lcd.print(voltage);

  lcd.setCursor(9,0);
  lcd.print("C:");
  lcd.print(current);


  lcd.setCursor(0,1);
  lcd.print("Temp :");
  lcd.print(event.temperature);

  if (event.temperature>50)
    digitalWrite(4, HIGH);
  
  else
    digitalWrite(4, LOW);
 
  delay(delayMS);
  delay(1000);
}