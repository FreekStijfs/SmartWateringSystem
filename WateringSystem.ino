//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//LIBRARIES
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Servo.h>

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//PINS
#define MULTIPLEXER_SELECT_PIN          D6
#define ANALOG_INPUT_PIN                A0
#define SDA_PIN                         D1
#define SCL_PIN                         D5
#define OLED_RESET_PIN                  D4
#define MOTOR_PIN                       D2
#define INTERNAL_LED_PIN                D0
#define FLASH_BUTTON_PIN                D3

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//SENSOR & ACTUATOR SETUP
Adafruit_SSD1306 display(128 ,64, &Wire, OLED_RESET_PIN);
Adafruit_BME280 bme;
Servo servo;

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//SENSOR INPUT
/* Sensor Reading Variables */ 
int moisture;
float temperature;
float pressure;
float humidity;
int light;

/* Timers for humidity sensor and humidity sensor stabilizing */
unsigned long lastMoistureReading;
unsigned long moistureReadingInterval = 1000 * 60 * 12;
unsigned long lastMultiplexerStabilizerTimer;
unsigned long multiplexerStabilizerTimerDelay = 100;
byte moistureTimerSwitch = LOW;

/* Reads moisture level after readings have been stabilized */
void moistureSensorInput(){
  if(millis() - lastMoistureReading >= moistureReadingInterval-multiplexerStabilizerTimerDelay){
    if(moistureTimerSwitch == LOW){
      digitalWrite(MULTIPLEXER_SELECT_PIN, HIGH);     
      moistureTimerSwitch = HIGH;
      lastMultiplexerStabilizerTimer = millis();     
    }

    if(millis() - lastMultiplexerStabilizerTimer >= multiplexerStabilizerTimerDelay){
      if(moistureTimerSwitch == HIGH){
        moisture = analogRead(ANALOG_INPUT_PIN);      
        digitalWrite(MULTIPLEXER_SELECT_PIN, LOW); 
        moistureTimerSwitch = LOW;
        MoistureReader();
        lastMoistureReading = millis();
      }
    }   
  }
}

/* Stores and calculates average of 6 last moisture readings */
int avgMoisture = 1000;
int moistureTreshold = 350;
int moistureReadings[6] = {1000, 1000, 1000, 1000, 1000, 1000};

void MoistureReader(){
  avgMoisture = 0;
    
  moistureReadings[5] = moistureReadings[4];
  moistureReadings[4] = moistureReadings[3];
  moistureReadings[3] = moistureReadings[2];
  moistureReadings[2] = moistureReadings[1];
  moistureReadings[1] = moistureReadings[0];
  moistureReadings[0] = moisture;

  for(int i = 0; i<6; i++){
    avgMoisture += moistureReadings[i];
  }

  avgMoisture = avgMoisture/6;
}

/* Timer for other sensors */
unsigned long lastSensorReadings;
unsigned long sensorReadingsInterval = 1000 * 5;

/* Reads other sensor input at an interval */
void otherSensorInput(){
  if(millis() - lastSensorReadings >= sensorReadingsInterval){
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    humidity = bme.readHumidity();

    //Make sure that light level readings are not moisture level readings
    if(moistureTimerSwitch == LOW){
      light = analogRead(ANALOG_INPUT_PIN);
    }

    lastSensorReadings = millis();
  }
}

/* Button */ 
int buttonState;            
int lastButtonState = HIGH;

unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 50;

bool CheckFlashButtonPress(){
  int reading = digitalRead(FLASH_BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        return true;
      }
    }
  }
  lastButtonState = reading;
  return false; 
}

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//UI LOGIC
#define SENSOR_SCREEN                   0
#define MOISTURE_SCREEN                 1
#define LAST_WATERING_SCREEN            2
byte screen = 0;

/* Cycle to next UI screen after an interval */
unsigned long lastScreenCycle;
unsigned long screenCycleInterval = 1000 * 10; //30 Seconds

void UIScreenCycle(){
  if(millis() - lastScreenCycle >= screenCycleInterval){
    screen = (screen + 1) % 3;
    lastScreenCycle = millis();
  }
}

/* Update UI screen after an interval */
unsigned long lastScreenUpdate;
unsigned long screenUpdateInterval = 1000;

void ScreenUpdate(){
  if(millis() - lastScreenUpdate >= screenUpdateInterval){
    switch(screen){
      case SENSOR_SCREEN:   
        SensorReadingsScreen();
        break;
      case MOISTURE_SCREEN:
        MoistureGraphScreen();
        break;
      case LAST_WATERING_SCREEN:
        WateringScreen();
        break;
    }
    lastScreenUpdate = millis();
  }
}

/* UI Symbols */
static const unsigned char PROGMEM light_logo_bmp[] =
{ B00000010, B01000000,
  B01000010, B01000010,
  B00100000, B00000100,
  B00000011, B11000000,
  B00001111, B11110000,
  B00001111, B11110000,
  B11011111, B11111011,
  B00011111, B11111000,
  B00011111, B11111000,
  B11011111, B11111011,
  B00001111, B11110000,
  B00001111, B11110000,
  B00000011, B11000000,
  B00100000, B00000100,
  B01000010, B01000010,
  B00000010, B01000000, };

static const unsigned char PROGMEM pressure_logo_bmp[] =
{ B00000000, B00000000,
  B00010000, B01001110,
  B00010000, B01000010,
  B00001000, B00100010,
  B00001000, B00100010,
  B00010000, B01000110,
  B00010000, B01000010,
  B00010000, B01000010,
  B00100000, B10000010,
  B00100000, B10000010,
  B00010000, B01000110,
  B00010000, B01000010,
  B01111101, B11110010,
  B00111000, B11100010,
  B00010000, B01001110,
  B00000000, B00000000 };

static const unsigned char PROGMEM humidity_logo_bmp[] =
{ B00000000, B00000000,
  B00000110, B00110010,
  B00000110, B00110100,
  B00001001, B00001000,
  B00001001, B00010110,
  B00010000, B10100110,
  B00010000, B10000000,
  B00100000, B01000000,
  B00100000, B01000000,
  B01000000, B00100000,
  B01000000, B00100000,
  B01000000, B00100000,
  B00100000, B01000000,
  B00010000, B10000000,
  B00001111, B00000000,
  B00000000, B00000000 };

static const unsigned char PROGMEM moisture_logo_bmp[] =
{ B00000000, B00000000,
  B00000111, B00001000,
  B00001000, B10011100,
  B00011100, B10110110,
  B01111111, B01011100,
  B01111111, B01001000,
  B01110111, B01001000,
  B01010101, B01011010,
  B01010101, B01001100,
  B01010101, B00101000,
  B01010101, B01111110,
  B01010101, B01000010,
  B01010101, B01000010,
  B01010101, B00100100,
  B00100010, B00011000,
  B00000000, B00000000 };

static const unsigned char PROGMEM temperature_logo_bmp[] =
{ B00000000, B00000000,
  B00001100, B00000000,
  B00010010, B01101110,
  B00010110, B01101000,
  B00010010, B00001000,
  B00010110, B00001000,
  B00010010, B00001110,
  B00010110, B00000000,
  B00010010, B00000000,
  B00010110, B00000000,
  B00010010, B00000000,
  B00100001, B00000000,
  B00100001, B00000000,
  B00100001, B00000000,
  B00011110, B00000000,
  B00000000, B00000000 };

static const unsigned char PROGMEM time_logo_bmp[] =
{ B00000000, B00000000,
  B00000011, B11000000,
  B00000100, B00100000,
  B00011000, B10011000,
  B00010000, B10001000,
  B00100000, B10000100,
  B01000000, B10000010,
  B01000001, B10000010,
  B01000111, B10000010,
  B01000000, B00000010,
  B00100000, B00000100,
  B00010000, B00001000,
  B00011000, B00011000,
  B00000100, B00100000,
  B00000011, B11000000,
  B00000000, B00000000 };

static const unsigned char PROGMEM automatic_logo_bmp[] =
{ B00000000, B00000000,
  B00000000, B00010000,
  B00000000, B00011000,
  B00000111, B11111100,
  B00001000, B00011000,
  B00010000, B00010000,
  B00100011, B11000000,
  B01000010, B01000010,
  B01000010, B01000010,
  B01000011, B11000010,
  B01000010, B01000010,
  B00100010, B01000100,
  B00010000, B00001000,
  B00001000, B00010000,
  B00000111, B11100000,
  B00000000, B00000000 };

static const unsigned char PROGMEM manual_logo_bmp[] =
{ B00000000, B00000000,
  B00000000, B00010000,
  B00000000, B00011000,
  B00000111, B11111100,
  B00001000, B00011000,
  B00010000, B00010000,
  B00100100, B01000000,
  B01000110, B11000010,
  B01000101, B01000010,
  B01000100, B01000010,
  B01000100, B01000010,
  B00100100, B01000100,
  B00010000, B00001000,
  B00001000, B00010000,
  B00000111, B11100000,
  B00000000, B00000000 };

/* Sensor Readings Screen */
void SensorReadingsScreen(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(18,8);
  display.print("SENSOR READINGS");
  display.drawBitmap(0,24,temperature_logo_bmp,16,16,1);
  display.setCursor(18,28);
  display.print(temperature);
  display.drawBitmap(0,45,light_logo_bmp,16,16,1);
  display.setCursor(18,49);
  display.print(light);
  display.drawBitmap(50,24,humidity_logo_bmp,16,16,1);
  display.setCursor(68,28);
  display.print(String(humidity) + "%");
  display.drawBitmap(50,45,pressure_logo_bmp,16,16,1);
  display.setCursor(68,49);
  display.print(String(pressure) + "hPa");

  display.display();
}

/* Moisture Graph Screen */
void MoistureGraphScreen(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  //Draw outlines of graph
  display.drawLine(20,2,20,43,SSD1306_WHITE);
  display.drawLine(20,43,125,43,SSD1306_WHITE);
  for(int i = 1; i <= 5; i++){
    display.drawPixel(20+(i*21),42, SSD1306_WHITE);
  }

  //Draw symbols & numbers for graph
  display.drawBitmap(2,16,moisture_logo_bmp,16,16,1);
  display.drawBitmap(65,46,time_logo_bmp,16,16,1);
  display.setCursor(1,3);
  display.print(F("100"));
  display.setCursor(13,36);
  display.print(F("0"));
  display.setCursor(20,46);
  display.print(F("1H AGO"));
  display.setCursor(108,46);
  display.print(F("NOW"));

  //Draw the threshold value with a dotted line
  int y = GetY(moistureTreshold);
  for(int i = 20; i<=125; i += 2){
    display.drawPixel(i,y,SSD1306_WHITE);
  }

  DrawGraph();
  
  display.display();
}

/* Draws the graph */
void DrawGraph(){
  for(int i = 0; i<=5; i++){
    if(moistureReadings[i] != 0){
      int x = GetX(i);;
      int y = GetY(moistureReadings[i]);

      DrawPoint(x,y,i);
          
      if(i!=0){
        int prev_x = GetX(i-1);
        int prev_y = GetY(moistureReadings[i-1]);

        display.drawLine(x,y,prev_x,prev_y,SSD1306_WHITE);
      }
    }
  }
}

/* Converts the moisture reading to the right x value on the OLED screen */
int GetX(int i){
  return 125 - (i * 21);
}

/* Converts the moisture reading to the right y value on the OLED screen */
int GetY(int i){
  float j =  i * 40 / 1023;
  return 42 - (int)j;
}

/* Draws a point that is 3 by 3 on the OLED screen*/
void DrawPoint(int x, int y, int i){
  if(i < 5){
    display.drawPixel(x-1,y-1,SSD1306_WHITE);
    display.drawPixel(x,y-1,SSD1306_WHITE);
    display.drawPixel(x+1,y-1,SSD1306_WHITE);
    display.drawPixel(x-1,y,SSD1306_WHITE);
    display.drawPixel(x,y,SSD1306_WHITE);
    display.drawPixel(x+1,y,SSD1306_WHITE);
    display.drawPixel(x-1,y+1,SSD1306_WHITE);
    display.drawPixel(x,y+1,SSD1306_WHITE);
    display.drawPixel(x+1,y+1,SSD1306_WHITE);
  }
  else{
    display.drawPixel(x+1,y-1,SSD1306_WHITE);
    display.drawPixel(x+1,y,SSD1306_WHITE);
    display.drawPixel(x+1,y+1,SSD1306_WHITE);
  }
}

/* Variables for last watering time */
int seconds = 0;
int minutes = 0;
int hours = 0;
unsigned long lastWateringTimer = 0;

void WateringScreen(){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(18,8);
  display.print("LAST WATERING");

  int lastWateringSeconds = (millis() - lastWateringTimer) / 1000;
  SecondsToSMH(lastWateringSeconds);

  display.setCursor(10,32);
  display.print(String(hours) + ":" + String(minutes) + ":" + String(seconds));

  display.display();
}

void SecondsToSMH(int t){
  seconds = t % 60;
  t = (t-seconds)/60;
  minutes = t % 60;
  t = (t-minutes)/60;
  hours = t;
}

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//SERVO CONTROL
unsigned long lastServoUpdate;
unsigned long servoUpdateDelay = 500;
int pos = 0;
byte watering = LOW;
int angle = 120;

void WaterPlant(){
  if(watering == LOW){
    for(pos = 0; pos <= angle; pos+= 1){
      servo.write(pos);

      if(pos == angle)
        watering = HIGH;

      delay(15);
    }
  }
}

void StopWaterPlant(){
  if(watering == HIGH){
    for(pos = angle; pos >= 0; pos-= 1){
      servo.write(pos);

      if(pos == 0){
        watering = LOW;
        servo.detach();
      }

      delay(15);
    }
  }
}

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//STATE MACHINE

class State{
  public:
    virtual ~State(){}
    virtual void enter(){};
    virtual State* run() = 0;
};

class Automatic_Checking: public State{
  public:
    virtual void enter() override;
    virtual State* run() override;
};

class Automatic_Watering: public State{
  public:
    virtual void enter() override;
    virtual State* run() override;
};

class Manual: public State{
  public:
    virtual void enter() override;
    virtual State* run() override;
};

static Automatic_Checking automatic_checking;
static Automatic_Watering automatic_watering;
static Manual manual;

State* Automatic_Checking::run(){
  moistureSensorInput();
  otherSensorInput();
  UIScreenCycle();
  ScreenUpdate();
  StopWaterPlant();

  if(CheckFlashButtonPress())
    return &manual;
  else if(avgMoisture <= moistureTreshold)
    return &automatic_watering;
  else
    return this;
}

void Automatic_Checking::enter(){
  digitalWrite(INTERNAL_LED_PIN, LOW);
  
  screen = SENSOR_SCREEN;
  screenUpdateInterval = 1000;
  lastScreenUpdate = millis();
  
  moistureReadingInterval = 1000 * 60 * 12;
  //lastMoistureReading = millis();
  
  sensorReadingsInterval = 1000 * 5;
  //lastSensorReadings = millis();

  Serial.println("AUTOMATIC_CHECKING");
}

State* Automatic_Watering::run(){
  moistureSensorInput();
  ScreenUpdate();
  WaterPlant();

  if(avgMoisture > moistureTreshold){
    lastWateringTimer = millis();
    lastServoUpdate = millis();
    return &automatic_checking;
  }
  else
    return this;
}

void Automatic_Watering::enter(){
  digitalWrite(INTERNAL_LED_PIN, LOW);
  
  screen = MOISTURE_SCREEN;
  screenUpdateInterval = 500;
  lastScreenUpdate = millis();
  
  moistureReadingInterval = 500;
  //lastMoistureReading = millis();
  
  servo.attach(MOTOR_PIN);
  lastServoUpdate = millis();

  Serial.println("AUTOMATIC_WATERING");
}

State* Manual::run(){
  moistureSensorInput();
  otherSensorInput();
  UIScreenCycle();
  ScreenUpdate();

  if(CheckFlashButtonPress())
    return &automatic_checking;
  else
    return this;
}

void Manual::enter(){
  digitalWrite(INTERNAL_LED_PIN, HIGH);
  
  screen = SENSOR_SCREEN;
  screenUpdateInterval = 1000;
  lastScreenUpdate = millis();
  
  moistureReadingInterval = 1000 * 60 * 12;
  //lastMoistureReading = millis();
  
  sensorReadingsInterval = 1000 * 5;
  //lastSensorReadings = millis();
  
  Serial.println("MANUAL");
}

bool isChanged = false;
State* state = &automatic_checking;
State* lastState = state;

void CheckStateChange(){
  if(isChanged)
    lastState->enter();
  state = lastState->run();
  isChanged = (state != lastState);
  lastState = state;
}

//-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.
//SETUP & LOOP

void setup() {
  Serial.begin(9600);

  /* Pin modes & pin setup */
  pinMode(MULTIPLEXER_SELECT_PIN, OUTPUT);
  digitalWrite(MULTIPLEXER_SELECT_PIN, LOW);
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(FLASH_BUTTON_PIN, INPUT_PULLUP);
  pinMode(INTERNAL_LED_PIN, OUTPUT);
  digitalWrite(INTERNAL_LED_PIN, LOW);

  /* I2C */
  Wire.begin(SDA_PIN, SCL_PIN);

  /* BME */
  unsigned status;
  status = bme.begin();

  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  
  /* OLED */
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.display();
  delay(1000);
  display.clearDisplay();
}

void loop() {
  CheckStateChange();
}
