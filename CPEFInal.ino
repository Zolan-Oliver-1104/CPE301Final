// CPE 301 Final Project
// Members: Oliver Zolan, Sutter Reynolds, Bennet Rau, Connor Marks
// Swamp Cooler

#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>
#include <RTClib.h>

#define DHTPIN 29
#define DHTTYPE DHT11

#define RDA 0x80
#define TBE 0x20

DHT dht(DHTPIN, DHTTYPE);

RTC_DS3231 rtc;

//Initilizing the stepper Motor
const int stepsperRevolution = 200;
Stepper stepper(stepsperRevolution, 10, 11, 12, 13);

//Setting pins for the LCD Monitor
LiquidCrystal lcd(30, 31, 32, 33, 34, 35);

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

enum machineStates {
  DISABLED, IDLE, ERROR, RUNNING
};
machineStates currentState = DISABLED;

//Defining pins for temp and water sensors
#define TEMPERATURE_THRESHOLD 20
#define WATER_THRESHOLD 100


//Button pins initilaization
const int waterLevelPin = A1;
const int powerButtonPin = 2;
const int stepperButtonPin = 3;

//light pins initilaization
const int yellowLEDPin = 53;
const int greenLEDPin = 51;
const int redLEDPin = 50;
const int blueLEDPin = 52;

//fan pin
const int fanMotorPin = 8;

float temperature, humidity;

void setup() {
  //Initializing the pins to establish communcation with RTC
  PORTD |= (1 << PD0) | (1 << PD1);
  rtc.begin();

  //Set the correct time on the RTC module
  DateTime now = DateTime(23, 7, 30, 0, 0, 0);
  rtc.adjust(now);

  DDRE |= (0x01 << greenLEDPin | 0x01 << yellowLEDPin);
  DDRG |= (0x01 << blueLEDPin);
  DDRH |= (0x01 << redLEDPin | 0x01 << waterLevelPin | 0x01 << fanMotorPin);

  adc_init();

  U0init(9600);
  dht.begin();

  lcd.begin(16, 2);

  stepper.setSpeed(stepsperRevolution);
}

//current state of circuit
void loop() {
  DateTime now = rtc.now();
  monitorTempHumidity();
  currentState = currentLog(currentState, temperature, humidity);

  switch(currentState) {
    case DISABLED:
      disabledState();
      break;
    case IDLE:
      idleState();
      break;
    case ERROR:
      errorState();
      break;
    case RUNNING:
      runningState();
      break;
  }
}

//monitor the humidty and temperature and display to LCD screen to be read
void monitorTempHumidity() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C  ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
  delay(100);
}

//function for when the DISABLED state is activated
void disabledState() {
  digitalWrite(yellowLEDPin, HIGH);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(blueLEDPin, LOW);
  
    if (digitalRead(powerButtonPin) == LOW) {
        currentState = IDLE;
        digitalWrite(yellowLEDPin, LOW);
        digitalWrite(greenLEDPin, HIGH);
    } else {
        currentState = ERROR;
        digitalWrite(yellowLEDPin, LOW);
        digitalWrite(redLEDPin, HIGH);
        lcd.print("Error: Low water");
        delay(2000);
    }
}

//function for when the IDLE state is activated
void idleState() {
  digitalWrite(yellowLEDPin, LOW);
  digitalWrite(greenLEDPin, HIGH);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(blueLEDPin, LOW);
  moveVentClockwise();

  if(checkWaterLevel == LOW) {
    currentState = ERROR;
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(redLEDPin, HIGH);
    lcd.print("Error: Low water");
    delay(2000);
  } else {
    currentState = IDLE;
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    lcd.clear();
  }

    if(digitalRead(powerButtonPin) == LOW) {
      setFan(true);
      currentState = RUNNING;
      digitalWrite(greenLEDPin, LOW);
      digitalWrite(blueLEDPin, HIGH);
    } else {
    currentState = ERROR;
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(redLEDPin, HIGH);
    lcd.print("Error: Low water");
    delay(2000);
  }
}

//function for when the ERROR state is activated
void errorState() {
  digitalWrite(yellowLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(blueLEDPin, LOW);
  monitorTempHumidity();

  if(checkWaterLevel == HIGH) {
    currentState = IDLE;
    digitalWrite(redLEDPin, LOW);
    digitalWrite(greenLEDPin, HIGH);
    lcd.clear();
  } else {
    currentState = ERROR;
    digitalWrite(redLEDPin, HIGH);
    lcd.print("Error: Low water");
    delay(2000);
  }

  if (digitalRead(powerButtonPin) == LOW) {
    currentState = IDLE;
    digitalWrite(redLEDPin, LOW);
    digitalWrite(greenLEDPin, HIGH);
    lcd.clear();
  }
}

//Function for when the RUNNING state is activated
void runningState() {
  digitalWrite(yellowLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(blueLEDPin, HIGH);
  monitorTempHumidity();
  
  if(checkWaterLevel == LOW) {
    currentState = ERROR;
    digitalWrite(blueLEDPin, LOW);
    digitalWrite(redLEDPin, HIGH);
    lcd.print("Error: Low water");
    delay(2000);
  } else {
    currentState = RUNNING;
    digitalWrite(blueLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    lcd.clear();
  }

  if (digitalRead(powerButtonPin) == LOW) {
    setFan(false);
    currentState = IDLE;
    digitalWrite(blueLEDPin, LOW);
    digitalWrite(greenLEDPin, HIGH);
  }
}

//Vent movement
void moveVentClockwise() {
  while (true) {
    stepper.step(stepsperRevolution);
  }
}

void moveVentCounterClockwise() {
  stepper.step(-stepsperRevolution);
}

//Water level function
int checkWaterLevel(int pin) {
  return PINH & (0x01 << pin);
}

//Fan function
void setFan(bool fan) {
  if(fan) {
    PORTH |= (0x01 << fanMotorPin);
  } else {
    PORTH &= ~(0x01 << fanMotorPin);
  }
}

// State machine function
machineStates currentLog(machineStates currentState, float temperature, float humidity) {
  Serial.print("Current State: ");
  Serial.println(currentState);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  switch (currentState) {
    case DISABLED:
      if (temperature > TEMPERATURE_THRESHOLD && humidity < WATER_THRESHOLD) {
        return IDLE;
      }
      break;

    case IDLE:
      if (temperature > TEMPERATURE_THRESHOLD && humidity < WATER_THRESHOLD) {
        return RUNNING;
      } else if (temperature < TEMPERATURE_THRESHOLD && humidity > WATER_THRESHOLD) {
        return ERROR;
      }
      break;

    case ERROR:
      if (temperature < TEMPERATURE_THRESHOLD && humidity > WATER_THRESHOLD) {
        return IDLE;
      }
      break;

    case RUNNING:
      if (temperature < TEMPERATURE_THRESHOLD && humidity > WATER_THRESHOLD) {
        return ERROR;
      } else if (temperature > TEMPERATURE_THRESHOLD && humidity < WATER_THRESHOLD) {
        return IDLE;
      }
      break;
  }

  return currentState;
}

void adc_init() {
  ADCSRA = 0x80;
  ADCSRB = 0x00;
  ADMUX = 0x40;
}

unsigned int adc_read(unsigned char read_adc) {
  ADCSRB &= 0xF7;
  ADCSRB |= (read_adc & 0x08);

  ADMUX &= 0xF8;
  ADMUX |= (read_adc & 0x07);

  ADCSRA |= 0x40;
  while (ADCSRA & 0x40) {}
  return ADC;
}

void U0init(unsigned long U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);

  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

unsigned char U0kbhit() {
  return (RDA & *myUCSR0A);
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata) {
  while (!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}