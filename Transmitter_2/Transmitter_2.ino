#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>
#include <FastLED.h>
#include <EEPROM.h>

#define EEPROM_SIZE 64  // plenty of space for your values

// ======= NRF24L01 Pin Definitions =======
#define CE_PIN 5
#define CSN_PIN 17
#define SCK_PIN 18
#define MOSI_PIN 23
#define MISO_PIN 19

// ======= LED Setup =======
#define LED_PIN 26
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ======= Buzzer =======
#define BUZZER_PIN 25

// ======= ADC Pins =======
#define CH1_PIN 34   //Trottle
#define CH2_PIN 35   //Yaw
#define CH3_PIN 36   //Pitch
#define CH4_PIN 39   //Roll
#define AUX1_PIN 14  // Toggle switch 1
#define AUX2_PIN 12  // Toggle switch 2
#define AUX3_PIN 4
#define AUX4_PIN 16
#define VBAT_PIN 27  // Battery voltage via voltage divider

#define ADDR_THR_MIN 0
#define ADDR_THR_MID 4
#define ADDR_THR_MAX 8

#define ADDR_YAW_MIN 12
#define ADDR_YAW_MID 16
#define ADDR_YAW_MAX 20

#define ADDR_ROLL_MIN 24
#define ADDR_ROLL_MID 28
#define ADDR_ROLL_MAX 32

#define ADDR_PIT_MIN 36
#define ADDR_PIT_MID 40
#define ADDR_PIT_MAX 44


// ======= LCD =======
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ======= NRF24 =======
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
RF24 radio(CE_PIN, CSN_PIN);

// ======= Data Structure =======
struct MyData {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte AUX1;
  byte AUX2;
  byte AUX3;
  byte AUX4;
};
MyData data;

float batteryVoltage = 0;
unsigned long lcdUpdateTime = 0;
const unsigned long lcdUpdateInterval = 10000;

unsigned long calibrateUpdateTime = 0;
const unsigned long calibrateUpdateInterval = 10000;

int trottleMin;
int trottleMid;
int trottleMax;

int yawMin;
int yawMid;
int yawMax;

int rollMin;
int rollMid;
int rollMax;

int pitchMin;
int pitchMid;
int pitchMax;

int yawTop;
int yawBottom;

int rollTop;
int rollBottom;

int pitchTop;
int pitchBottom;


// ======= Function to map joystick values =======
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle)
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return (reverse ? 255 - val : val);
}

// ======= Reset Data =======
void resetData() {
  data.ch1 = 127;
  data.ch2 = 127;
  data.ch3 = 127;
  data.ch4 = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
  data.AUX3 = 0;
  data.AUX4 = 0;
}

void saveCalibration() {

  EEPROM.put(ADDR_THR_MIN, trottleMin);
  EEPROM.put(ADDR_THR_MID, trottleMid);
  EEPROM.put(ADDR_THR_MAX, trottleMax);

  EEPROM.put(ADDR_YAW_MIN, yawMin);
  EEPROM.put(ADDR_YAW_MID, yawMid);
  EEPROM.put(ADDR_YAW_MAX, yawMax);

  EEPROM.put(ADDR_ROLL_MIN, rollMin);
  EEPROM.put(ADDR_ROLL_MID, rollMid);
  EEPROM.put(ADDR_ROLL_MAX, rollMax);

  EEPROM.put(ADDR_PIT_MIN, pitchMin);
  EEPROM.put(ADDR_PIT_MID, pitchMid);
  EEPROM.put(ADDR_PIT_MAX, pitchMax);

  EEPROM.commit();  // IMPORTANT on ESP32

  lcd.clear();
  lcd.backlight();
  String msg = " Calibration saved! ";
  for (int i = 0; i < msg.length(); i++) {
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  lcd.noBacklight();
  Serial.println("Calibration saved!");
}

void loadCalibration() {

  EEPROM.get(ADDR_THR_MIN, trottleMin);
  EEPROM.get(ADDR_THR_MID, trottleMid);
  EEPROM.get(ADDR_THR_MAX, trottleMax);

  EEPROM.get(ADDR_YAW_MIN, yawMin);
  EEPROM.get(ADDR_YAW_MID, yawMid);
  EEPROM.get(ADDR_YAW_MAX, yawMax);

  EEPROM.get(ADDR_ROLL_MIN, rollMin);
  EEPROM.get(ADDR_ROLL_MID, rollMid);
  EEPROM.get(ADDR_ROLL_MAX, rollMax);

  EEPROM.get(ADDR_PIT_MIN, pitchMin);
  EEPROM.get(ADDR_PIT_MID, pitchMid);
  EEPROM.get(ADDR_PIT_MAX, pitchMax);

  // If EEPROM is uninitialised, restore defaults
  if (trottleMid == 0 || yawMid == 0) {
    lcd.clear();
    lcd.backlight();
    String msg = " Stick values not found, loading defaults ";
    for (int i = 0; i < msg.length(); i++) {
      msg += msg[0];
      msg.remove(0, 1);
      lcd.setCursor(0, 0);
      lcd.print(msg.substring(0, 16));
      delay(100);
    }
    lcd.noBacklight();
    trottleMin = 4;
    trottleMid = 2048;
    trottleMax = 3220;

    yawMin = 200;
    yawMid = 1960;
    yawMax = 3200;

    rollMin = 360;
    rollMid = 1745;
    rollMax = 3240;

    pitchMin = 450;
    pitchMid = 1950;
    pitchMax = 3700;
  } else {
    lcd.clear();
    lcd.backlight();
    String msg = " Stick calibration values loaded ";
    for (int i = 0; i < msg.length(); i++) {
      msg += msg[0];
      msg.remove(0, 1);
      lcd.setCursor(0, 0);
      lcd.print(msg.substring(0, 16));
      delay(100);
    }
    lcd.noBacklight();
  }

  //Serial.println("Calibration loaded.");
}




void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);


  // ====== FastLED ======
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  //leds[0] = CRGB::Blue;
  //FastLED.show();

  // ====== Buzzer ======
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  tone(BUZZER_PIN, 600);
  delay(200);
  tone(BUZZER_PIN, 800);
  delay(400);
  tone(BUZZER_PIN, 1200);
  delay(200);
  noTone(BUZZER_PIN);

  // ====== ADC / Buttons ======
  pinMode(AUX1_PIN, INPUT_PULLUP);
  pinMode(AUX2_PIN, INPUT_PULLUP);
  pinMode(AUX3_PIN, INPUT_PULLUP);
  pinMode(AUX4_PIN, INPUT_PULLUP);

  // ====== LCD ======
  lcd.init();
  lcd.noBacklight();

  delay(500);
  // ====== NRF24 ======
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);


  if (!radio.begin()) {
    //Serial.println("NRF24L01 failed");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NRF24L01 failed");
    //while (1);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("NRF24L01 Initialized");
  }


  delay(500);

  //radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);  // Enable PA+LNA high power
  //radio.setChannel(108);            // Avoid Wi-Fi overlap
  radio.setDataRate(RF24_250KBPS);  // Long range stable link
  radio.openWritingPipe(0xE8E8F0F0E1LL);


  trottleMin = 4;
  trottleMid = 2048;
  trottleMax = 3220;

  yawMin = 200;
  yawMid = 1960;
  yawMax = 3200;

  rollMin = 360;
  rollMid = 1745;
  rollMax = 3240;

  pitchMin = 450;
  pitchMid = 1950;
  pitchMax = 3700;
  loadCalibration();
  setBounds();



  tone(BUZZER_PIN, 1200);
  delay(200);
  tone(BUZZER_PIN, 200);
  delay(400);
  noTone(BUZZER_PIN);


  resetData();
}

void loop() {
  unsigned long currentMillis = millis();

  unsigned long Millis1 = millis();


  // Average out values
  long read_CH1 = 0;
  long read_CH2 = 0;
  long read_CH3 = 0;
  long read_CH4 = 0;
  int num_samples = 12;
  for (int i = 0; i < num_samples; i++) {
    read_CH1 += analogRead(CH1_PIN);
    read_CH2 += analogRead(CH2_PIN);
    read_CH3 += analogRead(CH3_PIN);
    read_CH4 += analogRead(CH4_PIN);
  }
  float average_CH1 = read_CH1 / num_samples;
  float average_CH2 = read_CH2 / num_samples;
  float average_CH3 = read_CH3 / num_samples;
  float average_CH4 = read_CH4 / num_samples;

  data.ch1 = mapJoystickValues(average_CH1, trottleMax, trottleMid, trottleMin, true);  //Throttle
  data.ch2 = mapJoystickValues(average_CH2, yawMax, yawMid, yawMin, true);              //Yaw
  data.ch3 = mapJoystickValues(average_CH3, pitchMax, pitchMid, pitchMin, true);        //Pitch
  data.ch4 = mapJoystickValues(average_CH4, rollMax, rollMid, rollMin, true);           //Roll
  // ====== Read Inputs ======
  data.AUX1 = digitalRead(AUX1_PIN) == LOW ? 1 : 0;
  data.AUX2 = digitalRead(AUX2_PIN) == LOW ? 1 : 0;
  data.AUX3 = digitalRead(AUX3_PIN) == LOW ? 1 : 0;
  data.AUX4 = digitalRead(AUX4_PIN) == LOW ? 1 : 0;

  if (data.ch2 <= yawTop && data.ch2 >= yawBottom) {
    data.ch2 = 127;
  }

  if (data.ch3 <= pitchTop && data.ch3 >= pitchBottom) {
    data.ch3 = 127;
  }

  if (data.ch4 <= rollTop && data.ch4 >= rollBottom) {
    data.ch4 = 127;
  }

  //Calibrating sticks
  if (average_CH3 < 1500 && average_CH4 < 1500 && currentMillis - calibrateUpdateTime >= calibrateUpdateInterval && average_CH1 > 2900) {
    calibrateSticks();
    setBounds();
    calibrateUpdateTime = currentMillis;
  }

  /*
  Serial.print(" trottleMin: ");
  Serial.print(trottleMin);
  Serial.print(" trottleMax: ");
  Serial.print(trottleMax);
  Serial.print(" yawMin: ");
  Serial.print(yawMin);
  Serial.print(" yawMid: ");
  Serial.print(yawMid);
  Serial.print(" yawMax: ");
  Serial.print(yawMax);
  Serial.print(" rollMin: ");
  Serial.print(rollMin);
  Serial.print(" rollMid: ");
  Serial.print(rollMid);
  Serial.print(" rollMax: ");
  Serial.print(rollMax);
  */

  Serial.print(" Channel 1: ");
  Serial.print(data.ch1);
  Serial.print(" Channel 2: ");
  Serial.print(data.ch2);
  Serial.print(" Channel 3: ");
  Serial.print(data.ch3);
  Serial.print(" Channel 4: ");
  Serial.print(data.ch4);

  /*
  Serial.print(" Aux 1: ");
  Serial.print(data.AUX1);
  Serial.print(" Aux 2: ");
  Serial.print(data.AUX2);
  Serial.print(" Aux 3: ");
  Serial.print(data.AUX3);
  Serial.print(" Aux 4: ");
  Serial.print(data.AUX4);
  */


  // ====== Send over NRF ======
  radio.write(&data, sizeof(MyData));


  // ====== Update LCD every 500ms ======
  if (currentMillis - lcdUpdateTime >= lcdUpdateInterval) {
    int raw = analogRead(VBAT_PIN);
    batteryVoltage = (raw / 4095.0) * 3.3 * 2;  // Assuming voltage divider x2
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Batt: ");
    lcd.print(batteryVoltage, 2);
    lcd.print("V");


    lcdUpdateTime = currentMillis;
  };

  // ====== LED Indicator ======
  //leds[0] = CRGB(255, 100, 50);  // Orange-like
  //FastLED.show();

  unsigned long Millis2 = millis();
  unsigned long TimeDiff = Millis2 - Millis1;

  Serial.print(" Time: ");
  Serial.println(TimeDiff);
}


void calibrateSticks() {
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sticks cal start");
  delay(800);

  // ===============================
  // Helper lambda for scrolling text
  // ===============================
  auto scrollMsg = [&](String msg) {
    for (int i = 0; i < msg.length(); i++) {
      msg += msg[0];
      msg.remove(0, 1);
      lcd.setCursor(0, 0);
      lcd.print(msg.substring(0, 16));
      delay(500);
    }
  };

  // ===============================
  // Helper: Average 64 samples
  // ===============================
  auto readAvg = [&](int pin) {
    float t = 0;
    for (int i = 0; i < 12; i++) {
      t += analogRead(pin);
    }
    return t / 12.0f;
  };



  // ===============================
  // THROTTLE MIN
  // ===============================
  lcd.clear();
  String msg = " Move throttle to minimum position ";
  while (analogRead(CH1_PIN) < 2400) {
    //scrollMsg(" Move throttle to minimum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  trottleMin = readAvg(CH1_PIN) - 250;

  // ===============================
  // THROTTLE MAX
  // ===============================
  lcd.clear();
  msg = " Move throttle to maximum position ";
  while (analogRead(CH1_PIN) > 1500) {
    //scrollMsg(" Move throttle to maximum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  trottleMax = readAvg(CH1_PIN) + 250;

  // ===============================
  // YAW MIN
  // ===============================
  lcd.clear();
  msg = " Move yaw to minimum position ";
  while (analogRead(CH2_PIN) < 2400) {
    //scrollMsg(" Move yaw to minimum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  yawMin = readAvg(CH2_PIN) - 250;

  // ===============================
  // YAW MID
  // ===============================
  lcd.clear();
  msg = " Move yaw to centre position ";
  while (analogRead(CH2_PIN) < 1500 || analogRead(CH2_PIN) > 2500) {
    //scrollMsg(" Move yaw to centre position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  yawMid = readAvg(CH2_PIN);

  // ===============================
  // YAW MAX
  // ===============================
  lcd.clear();
  msg = " Move yaw to maximum position ";
  while (analogRead(CH2_PIN) > 1500) {
    //scrollMsg(" Move yaw to maximum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  yawMax = readAvg(CH2_PIN) + 250;

  // ===============================
  // ROLL MIN
  // ===============================
  lcd.clear();
  msg = " Move roll to minimum position ";
  while (analogRead(CH4_PIN) < 2400) {
    //scrollMsg(" Move roll to minimum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  rollMin = readAvg(CH4_PIN) - 250;

  // ===============================
  // ROLL MID
  // ===============================
  lcd.clear();
  msg = " Move roll to centre position ";
  while (analogRead(CH4_PIN) < 1500 || analogRead(CH4_PIN) > 2500) {
    //scrollMsg(" Move roll to centre position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  rollMid = readAvg(CH4_PIN);

  // ===============================
  // ROLL MAX
  // ===============================
  lcd.clear();
  msg = " Move roll to maximum position ";
  while (analogRead(CH4_PIN) > 1500) {
    //scrollMsg(" Move roll to maximum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  rollMax = readAvg(CH4_PIN) + 250;

  // ===============================
  // PITCH MIN
  // ===============================
  lcd.clear();
  msg = " Move pitch to minimum position ";
  while (analogRead(CH3_PIN) < 2400) {
    //scrollMsg(" Move pitch to minimum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  pitchMin = readAvg(CH3_PIN) - 250;

  // ===============================
  // PITCH MID
  // ===============================
  lcd.clear();
  msg = " Move pitch to centre position ";
  while (analogRead(CH3_PIN) < 1500 || analogRead(CH3_PIN) > 2500) {
    //scrollMsg(" Move pitch to centre position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  pitchMid = readAvg(CH3_PIN);

  // ===============================
  // PITCH MAX
  // ===============================
  lcd.clear();
  msg = " Move pitch to maximum position ";
  while (analogRead(CH3_PIN) > 1500) {
    //scrollMsg(" Move pitch to maximum position ");
    msg += msg[0];
    msg.remove(0, 1);
    lcd.setCursor(0, 0);
    lcd.print(msg.substring(0, 16));
    delay(500);
  }
  delay(5000);
  pitchMax = readAvg(CH3_PIN) + 250;

  trottleMid = 2048;

  // ===============================
  // DONE
  // ===============================
  lcd.clear();
  lcd.setCursor(0, 0);
  saveCalibration();
  lcd.print("Cal complete!");
  lcd.noBacklight();
  delay(2000);
}

void setBounds() {
  float rollAve = 0;
  float pitchAve = 0;
  float yawAve = 0;

  for (int i = 0; i < 12; i++) {
    yawAve += mapJoystickValues(analogRead(CH2_PIN), yawMax, yawMid, yawMin, true);          //Yaw
    pitchAve += mapJoystickValues(analogRead(CH3_PIN), pitchMax, pitchMid, pitchMin, true);  //Pitch
    rollAve += mapJoystickValues(analogRead(CH4_PIN), rollMax, rollMid, rollMin, true);      //Roll
  }

  yawTop = (yawAve / 12) + 8;
  yawBottom = (yawAve / 12) - 8;

  pitchTop = (pitchAve / 12) + 8;
  pitchBottom = (pitchAve / 12) - 8;

  rollTop = (rollAve / 12) + 8;
  rollBottom = (rollAve / 12) - 8;

}
