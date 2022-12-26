
#include <DHT.h>
#include"esp32-hal-ledc.h"
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Wire.h>
#define DHTTYPE DHT11  
#define pwmPin_1 14
#define  OUT_OF_MEASUREMENT_RANGE -1

// Replace with your network credentials
const char* ssid = "P502";
const char* password = "23456789";
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
uint8_t DHTPin = 18; 
DHT dht(DHTPin, DHTTYPE);   


int data_count = 1000;
double data_total ;
double time_data = 0;
double data_ave;
double Vref = 3.3;
double calib_volt;
double resistance = 10.0;
double temp_resistance;
double temp;

double temp_1;

const int pwmChannel_1 = 0;
const int pwmChannel_2 = 1;
const int freq = 12800;
const int resultion = 8;


double kp = 30.00;
double ki = 0;
double kd = 0.1;
double targetTemp = 27.00;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double cumError, rateError;
double pwmOut;
int pwm = 0;




String getDistance() {


// Initialize DHT sensor.
            
  float Temperature;
  Temperature = dht.readTemperature();

  if (isnan(Temperature)) {
    Serial.println("Failed to read from DHT-11 sensor!");
    return "";
  }
  else {
    Serial.println(Temperature);
    return String(Temperature);
  }
}
String getDistance1() {


// Initialize DHT sensor.
            
  float Humidity;
  Humidity = dht.readHumidity();

  if (isnan(Humidity)) {
    Serial.println("Failed to read from DHT-11 sensor!");
    return "";
  }
  else {
    Serial.println(Humidity);
    return String(Humidity);
  }
}

void setup () {
  // Serial port for debugging purposes
  Serial.begin (115200);
  delay(100);
  pinMode(pwmPin_1, OUTPUT);
  ledcSetup(pwmChannel_1, freq, resultion);
  ledcAttachPin(pwmPin_1, pwmChannel_1);
  
  pinMode(DHTPin, INPUT);

  dht.begin();   
      
  // Initialize SPIFFS
  if (! SPIFFS.begin ()) {
    Serial.println ("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Route for web page
  server.on ("/", HTTP_GET, [] (AsyncWebServerRequest * request) {
    request-> send (SPIFFS, "/index.html");
  });
  server.on ("/distance", HTTP_GET, [] (AsyncWebServerRequest * request) {
    request-> send_P (200, "text / plain", getDistance(). c_str ());
  });
   server.on ("/humidity", HTTP_GET, [] (AsyncWebServerRequest * request) {
    request-> send_P (200, "text / plain", getDistance1(). c_str ());
  });


  // start server
  server.begin ();
}
void loop() {
  time_data = micros() / 1000000.00;
  temp_1=dht.readTemperature();
  
  pwmOut = biologicalTempControl(temp_1);
  Serial.println(pwmOut);
   Serial.println(temp_1);
  if (pwmOut > 0) {
    ledcWrite(pwmChannel_1, 0);
  }
  else {
    ledcWrite(pwmChannel_1, abs(pwmOut));
  }

}
double biologicalTempControl(double currentTemp) {
  currentTime = millis();

  elapsedTime = currentTime - previousTime;
  error = targetTemp - currentTemp;

  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;

  double out = kp * error + ki * cumError + kd * rateError;

  lastError = error;
  previousTime = currentTime;

  out = constrain(out, -255, 255);
  return out;
}
