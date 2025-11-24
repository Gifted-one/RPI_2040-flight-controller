#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include "hardware/pwm.h"  // For RP2040 PWM control
#include <Wire.h>
#include <Servo.h>  // Using the standard Arduino Servo library which works with Pico
#include <EEPROM.h>


// Define EEPROM addresses for each variable
#define EEPROM_SIZE 512  // Use first 512 bytes of flash

// EEPROM addresses for each calibration value
const int ADDR_ROLL = 0;
const int ADDR_PITCH = sizeof(float);
const int ADDR_YAW = sizeof(float) * 2;
const int ADDR_ACC_X = sizeof(float) * 3;
const int ADDR_ACC_Y = sizeof(float) * 4;
const int ADDR_ACC_Z = sizeof(float) * 5;

volatile float RatePitch,
  RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

float PAngleRoll = 2;
float PAnglePitch = 1.5;
float IAngleRoll = 0.5;  //Changed
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.006;
float DAnglePitch = 0.002;

float PRateRoll = 3.2;   //Changed
float IRateRoll = 0.75;  //Changed
float DRateRoll = 0.05;  //Changed

float PRatePitch = 2.1;  //Pitch axis;
float IRatePitch = 0.5;
float DRatePitch = 0.06;  //Pitch axis;

float PRateYaw = 5;  //Changed
float IRateYaw = 0;  //Changed
float DRateYaw = 0;

uint32_t LoopTimer;
float dt = 0.004;  // time cycle (4ms = 250Hz)
float t = 0.004;   // time cycle (4ms = 250Hz)


// Pico pin assignments - adjust these based on your wiring
const int mot1_pin = 6;
const int mot2_pin = 3;
const int mot3_pin = 7;
const int mot4_pin = 2;

bool autoLevel = false;
bool armed = false;

int ThrottleIdle = 1070;
int ThrottleCutOff = 1000;

unsigned long armedUpdateTime = 0;
const unsigned long armedUpdateInterval = 2000;

int dbgCount = 0;
float dbgERollSum = 0, dbgEPitchSum = 0, dbgEYawSum = 0;


// ------------------- PWM helpers -------------------
//const int escPins[4] = { 3, 6, 7, 2 };  // GPIO pins for motors
const int pwmFreq = 400;     // ESC frequency in Hz
const int pwmRes = 16;       // 16-bit resolution (0–65535)
const int pwmRange = 65535;  // Max duty for analogWrite()


#define NRF_CS 17
#define NRF_CE 14


RF24 radio(NRF_CE, NRF_CS);

// Create the same data structure as transmitter
struct Received_data {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte AUX1;
  byte AUX2;
};

Received_data received_data;

struct PT1 {
  float a = 0, y = 0;
  void config(float cutoff_hz, float dt) {
    float rc = 1.0f / (2.0f * 3.1415926f * cutoff_hz);
    a = dt / (rc + dt);
  }
  float step(float x) {
    y += a * (x - y);
    return y;
  }
};


// Filters for gyro and D-term
PT1 gyroLPF_R, gyroLPF_P, gyroLPF_Y;
PT1 dtermLPF_R, dtermLPF_P, dtermLPF_Y;



volatile int ReceiverValue[6];  // For Channel 1 to Channel 6


volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = { 0, 0, 0 };

// Angle calculation variables
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

unsigned long calibrateUpdateTime = 0;
const unsigned long calibrateUpdateInterval = 10000;

// Convert microseconds (1000–2000) into duty cycle value
uint16_t usToDuty(int us) {
  // One 400 Hz cycle = 2500 µs period
  float duty = (float)us / 2500.0f;
  return (uint16_t)(duty * pwmRange);
}



void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  // Initialize LED pin (adjust as needed)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize I2C for MPU6050
  Wire.setSDA(4);  // Default I2C0 pins for Pico
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU6050
  delay(100);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // DLPF config
  Wire.write(0x03);  // Bandwidth 44Hz
  Wire.endTransmission();

  // Improved NRF24 initialization
  if (radio.begin()) {
    Serial.println("NRF24L01 initialized successfully");
  } else {
    Serial.println("NRF24L01 initialization failed");
    //while (1);
  }

  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_LOW);    // RF24_PA_LOW, RF24_PA_HIGH, or RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS);  // RF24_250KBPS, RF24_1MBPS, or RF24_2MBPS

  // Match the pipe address from transmitter (0xE8E8F0F0E1LL)
  const uint64_t pipe = 0xE8E8F0F0E1LL;
  radio.openReadingPipe(1, pipe);
  radio.startListening();


  // Init motors--Non-fixed frequency
  //initMotors();                // setup ESCs
  analogWriteFreq(pwmFreq);    // Set PWM frequency
  analogWriteRange(pwmRange);  // Set duty cycle range

  pinMode(mot1_pin, OUTPUT);
  pinMode(mot2_pin, OUTPUT);
  pinMode(mot3_pin, OUTPUT);
  pinMode(mot4_pin, OUTPUT);

  analogWrite(mot1_pin, usToDuty(1000));
  analogWrite(mot2_pin, usToDuty(1000));
  analogWrite(mot3_pin, usToDuty(1000));
  analogWrite(mot4_pin, usToDuty(1000));

  delay(2000);  // Give ESCs time to initialize


  RateCalibrationRoll = 6.02;
  RateCalibrationPitch = 5.30;
  RateCalibrationYaw = -0.33;
  AccXCalibration = 0.06;
  AccYCalibration = -0.02;
  AccZCalibration = 0.01;

  // Try to load existing calibrations, use defaults if not found
  loadCalibrations();

  LoopTimer = micros();

  gyroLPF_R.config(70.0f, dt);
  gyroLPF_P.config(70.0f, dt);
  gyroLPF_Y.config(70.0f, dt);

  dtermLPF_R.config(30.0f, dt);
  dtermLPF_P.config(30.0f, dt);
  dtermLPF_Y.config(25.0f, dt);  // yaw usually no D
}


void gyro_signals(void) {
  // Configure MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // DLPF config
  Wire.write(0x03);  // Bandwidth 44Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  // Accelerometer config
  Wire.write(0x10);  // ±8g
  Wire.endTransmission();

  // Read accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Accelerometer data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Configure gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // Gyro config
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission();

  // Read gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // Gyro data register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw data to physical values
  RateRoll = (float)GyroX / 65.5;  // 65.5 LSB/°/s for ±500°/s range
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;  // 4096 LSB/g for ±8g range
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
}

void loadCalibrations() {
  // Try to read existing calibrations
  EEPROM.get(ADDR_ROLL, RateCalibrationRoll);
  EEPROM.get(ADDR_PITCH, RateCalibrationPitch);
  EEPROM.get(ADDR_YAW, RateCalibrationYaw);
  EEPROM.get(ADDR_ACC_X, AccXCalibration);
  EEPROM.get(ADDR_ACC_Y, AccYCalibration);
  EEPROM.get(ADDR_ACC_Z, AccZCalibration);

  // Check if values were previously saved (not all zeros)
  if (isnan(RateCalibrationRoll) || RateCalibrationRoll == 0.0) {
    Serial.println("No previous calibrations found, using defaults");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    // Reset to defaults
    RateCalibrationRoll = 6.02;
    RateCalibrationPitch = 5.30;
    RateCalibrationYaw = -0.33;
    AccXCalibration = 0.06;
    AccYCalibration = -0.02;
    AccZCalibration = 0.01;
  } else {
    Serial.println("Calibrations loaded from EEPROM");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void saveCalibrations() {
  EEPROM.put(ADDR_ROLL, RateCalibrationRoll);
  EEPROM.put(ADDR_PITCH, RateCalibrationPitch);
  EEPROM.put(ADDR_YAW, RateCalibrationYaw);
  EEPROM.put(ADDR_ACC_X, AccXCalibration);
  EEPROM.put(ADDR_ACC_Y, AccYCalibration);
  EEPROM.put(ADDR_ACC_Z, AccZCalibration);

  if (EEPROM.commit()) {
    Serial.println("Calibrations saved to EEPROM successfully!");

  } else {
    Serial.println("ERROR: Failed to save calibrations!");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void calibrateGyroSimple() {
  Serial.println("Calibrating IMU");
  //delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  for (int RateCalibrationNumber = 0; RateCalibrationNumber < 5000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    AccXCalibration += AccX;
    AccYCalibration += AccY;
    AccZCalibration += AccZ;

    delay(1);
  }
  RateCalibrationRoll /= 5000;
  RateCalibrationPitch /= 5000;
  RateCalibrationYaw /= 5000;
  AccXCalibration /= 5000;
  AccYCalibration /= 5000;
  AccZCalibration /= 5000;
  AccZCalibration = AccZCalibration - 1;

  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  saveCalibrations();
  Serial.println("Calibrating IMU finished");
  //delay(2000);
}

void loop() {

  unsigned long currentMillis = millis();
  uint32_t now = micros();
  // Read IMU data
  gyro_signals();

  if (radio.available()) {
    radio.read(&received_data, sizeof(Received_data));

    ReceiverValue[0] = map(received_data.ch4, 0, 255, 1000, 2000);  //Roll
    ReceiverValue[1] = map(received_data.ch3, 0, 255, 1000, 2000);  //Pitch
    ReceiverValue[2] = map(received_data.ch1, 0, 255, 1000, 2000);  //Trottle
    ReceiverValue[3] = map(received_data.ch2, 0, 255, 1000, 2000);  //Yaw
    ReceiverValue[4] = map(received_data.AUX1, 0, 1, 1000, 2000);
    ReceiverValue[5] = map(received_data.AUX2, 0, 1, 1000, 2000);


    if (ReceiverValue[0] >= 1498 && ReceiverValue[0] <= 1500) {
      ReceiverValue[0] = 1500;
    }
    if (ReceiverValue[1] >= 1498 && ReceiverValue[1] <= 1500) {
      ReceiverValue[1] = 1500;
    }

    if (ReceiverValue[3] >= 1498 && ReceiverValue[3] <= 1500) {
      ReceiverValue[3] = 1500;
    }


    //Arming the motors
    if (ReceiverValue[3] < 1200 && ReceiverValue[2] < 1200 && currentMillis - armedUpdateTime >= armedUpdateInterval) {
      armed = !armed;
      armedUpdateTime = currentMillis;
    }
  }

  //Serial.print(" Pitch ");
  //Serial.print(ReceiverValue[1]);
  //Serial.print(" Roll ");
  //Serial.print(ReceiverValue[0]);
  //Serial.print(" Yaw ");
  //Serial.print(ReceiverValue[3]);


  if (!armed && ReceiverValue[0] < 1070 && ReceiverValue[1] < 1070 && currentMillis - calibrateUpdateTime >= calibrateUpdateInterval && radio.available()) {
    calibrateGyroSimple();
    calibrateUpdateTime = currentMillis;
  }



  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Filter gyro for P/I
  float RateRoll_f = gyroLPF_R.step(RateRoll);
  float RatePitch_f = gyroLPF_P.step(RatePitch);
  float RateYaw_f = gyroLPF_Y.step(RateYaw);

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;


  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;


  complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll_f * dt) + 0.009 * AngleRoll;
  complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch_f * dt) + 0.009 * AnglePitch;

  //complementaryAnglePitch -= 1;

  // Clamping complementary filter roll angle to ±20 degrees
  complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);



  /*
  dbgERollSum += complementaryAngleRoll;
  dbgEPitchSum += complementaryAnglePitch;
  dbgCount++;
  if (dbgCount >= 250) {
    Serial.print("MeanErr Roll:");
    Serial.print(dbgERollSum / dbgCount);
    Serial.print("MeanErr Pitch:");
    Serial.println(dbgEPitchSum / dbgCount);
    dbgCount = 0;
    dbgERollSum = dbgEPitchSum = 0;
  }
  */
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = -0.1 * (ReceiverValue[3] - 1500);
  ;

  //Added auto leveling feature
  if (ReceiverValue[4] == 2000) {
    autoLevel = true;
  } else {
    autoLevel = false;
  }

  dbgERollSum += complementaryAngleRoll;
  dbgEPitchSum += complementaryAnglePitch;
  //dbgEYawSum += ErrorRateYaw;
  dbgCount++;
  if (dbgCount >= 250) {
    Serial.print("MeanErr Roll: ");
    Serial.print(dbgERollSum / dbgCount);
    Serial.print(" MeanErr Pitch: ");
    Serial.println(dbgEPitchSum / dbgCount);
    //Serial.print(" MeanErr Yaw: ");
    //Serial.println(dbgEYawSum / dbgCount);
    dbgCount = 0;
    dbgERollSum = dbgEPitchSum = dbgEYawSum = 0;
  }

  if (autoLevel) {

    //DesiredRateRoll = 0.1 * ((ReceiverValue[0] - complementaryAngleRoll * 15) - 1500);
    //DesiredRatePitch = 0.1 * ((ReceiverValue[1] - complementaryAnglePitch * 15) - 1500);
    //DesiredRateRoll = 0.1 * (ReceiverValue[0] - 1500) - 2 * (complementaryAngleRoll);
    //DesiredRatePitch = 0.1 * (ReceiverValue[1] - 1500) - 1.5 * (complementaryAnglePitch);
    //DesiredRateRoll = 0;
    //DesiredRatePitch = 0;
    
    DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);

    // Inlined PID equation for Roll
    ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
    PtermRoll = PAngleRoll * ErrorAngleRoll;
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (dt / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / dt);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
    DesiredRateRoll = PIDOutputRoll;
    PrevErrorAngleRoll = ErrorAngleRoll;
    PrevItermAngleRoll = ItermRoll;

    ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch + 1;
    PtermPitch = PAnglePitch * ErrorAnglePitch;
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (dt / 2));
    ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / dt);
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
    PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
    DesiredRatePitch = PIDOutputPitch;
    PrevErrorAnglePitch = ErrorAnglePitch;
    PrevItermAnglePitch = ItermPitch;
    

  } else {
    DesiredRateRoll = 0.1 * (ReceiverValue[0] - 1500);
    DesiredRatePitch = 0.1 * (ReceiverValue[1] - 1500);
    //DesiredRateRoll = 0;
    //DesiredRatePitch = 0;
  }

  ErrorRateRoll = DesiredRateRoll - RateRoll_f + 1.80;
  ErrorRatePitch = DesiredRatePitch - RatePitch_f;
  ErrorRateYaw = DesiredRateYaw - RateYaw_f;




  // Roll Axis PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (dt / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);

  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / dt);

  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

  // Update output and previous values for Roll
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Axis PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (dt / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);

  //DtermPitch = DRatePitch * (-(RatePitch_d - PrevErrorRatePitch) / dt);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / dt);

  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

  // Update output and previous values for Pitch
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Axis PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (dt / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / dt);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]


  // Update output and previous values for Yaw
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;


  if (InputThrottle > 2000) {
    InputThrottle = 1900;
  }


  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw);  // front right - counter clockwise
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw);  // rear right - clockwise
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw);  // rear left  - counter clockwise
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw);  //front left - clockwise


  if (MotorInput1 > 2000) {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000) {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000) {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000) {
    MotorInput4 = 1999;
  }


  //int ThrottleIdle = 1150;
  // int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle) {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle) {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle) {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle) {
    MotorInput4 = ThrottleIdle;
  }

  if (!armed)  //Arm the motors
  {

    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
  } else {

    if (ReceiverValue[2] < 1200) {
      MotorInput1 = 1200;
      MotorInput2 = 1200;
      MotorInput3 = 1200;
      MotorInput4 = 1200;
    }
  }

  if (ReceiverValue[2] < 1030) {

    //MotorInput1 = ThrottleCutOff;
    //MotorInput2 = ThrottleCutOff;
    //MotorInput3 = ThrottleCutOff;
    //MotorInput4 = ThrottleCutOff;
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;
  }

  analogWrite(mot1_pin, usToDuty(MotorInput1));
  analogWrite(mot2_pin, usToDuty(MotorInput2));
  analogWrite(mot3_pin, usToDuty(MotorInput3));
  analogWrite(mot4_pin, usToDuty(MotorInput4));

  //Serial.print(" MotVals-");
  //Serial.print(MotorInput1);
  //Serial.print("  ");
  //Serial.print(MotorInput2);
  //Serial.print("  ");
  //Serial.print(MotorInput3);
  //Serial.print("  ");
  //Serial.print(MotorInput4);

  //Serial.print("  ");
  //Serial.print(ErrorRateRoll);
  //Serial.print("  ");
  //Serial.print(ErrorRatePitch);
  //Serial.print("  ");
  //Serial.print(ErrorRateYaw);

  //Serial.print(" complementaryAnglePitch: ");
  //Serial.print(complementaryAnglePitch);
  //Serial.print(" complementaryAngleRoll: ");
  //Serial.print(complementaryAngleRoll);

  //Serial.print(" Armed: ");
  //Serial.print(armed);
  //Serial.print(" Auto Level: ");
  //Serial.println(autoLevel);

  // Maintain loop timing (250Hz)
  // Measure dt in seconds
  while (micros() - LoopTimer < (dt * 1000000))
    ;
  {
    LoopTimer = micros();
  }
}