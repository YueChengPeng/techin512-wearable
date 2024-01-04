#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)  // standard air pressure: sea level

Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define MOTOR_PIN A0
#define DECAY_FACTOR 0.1  // Adjust the decay factor based on your requirements

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MPU_window_length = 10;     // filter window length
int MPU_readings[MPU_window_length];  // the readings from the analog input
int MPU_readIndex = 0;                // the index of the current reading
int MPU_total = 0;                    // the running total
int MPU_average = 0;                  // the average

#define MPU_WAVE_THRESHOLD 9.0  // Set a threshold for detecting the peak-to-valley delta
bool isRising = true;            // Flag for rising or falling edge
float lastPeak = 0;              // Record the last peak value
float lastValley = 0;            // Record the last valley value
int waveCount = 0;               // Counter for the number of waveform periods
bool this_wave_not_counted = true;
float filteredTemperature = -1;

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN, OUTPUT);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      ;  // Don't proceed, loop forever
  }

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000);  // Pause for 2 seconds

  for (int i = 0; i < MPU_window_length; i++) {
    MPU_readings[i] = 0;
  }

  // Clear the buffer
  display.clearDisplay();
}

void printValues() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.print("T=");
  float currentTemperature = bme.readTemperature();
  if(filteredTemperature == -1)
    filteredTemperature = currentTemperature;

  // Exponential decay filter
  filteredTemperature = (1 - DECAY_FACTOR) * filteredTemperature + DECAY_FACTOR * currentTemperature;

  display.println(filteredTemperature);

  // if (bme.readTemperature() > 30.0) {
  //   digitalWrite(MOTOR_PIN, HIGH);
  // } else {
  //   digitalWrite(MOTOR_PIN, LOW);
  // }
  // display.println("oC");

  // Convert temperature to Fahrenheit
  Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");

  display.print("Count=");
  display.print(waveCount / 2);


  display.display();
}

void countWavePeriod(float currentValue) {
  static float lastValue = -10000;

  // Check for the rising edge
  if (currentValue > lastValue) {
    if (!isRising) {
      isRising = true;
      if (lastPeak - lastValley > MPU_WAVE_THRESHOLD)
        waveCount++;
    }

    lastPeak = currentValue;
  }
  // Check for the falling edge
  else if (currentValue < lastValue) {
    if (isRising) {
      isRising = false;
      if (lastPeak - lastValley > MPU_WAVE_THRESHOLD)
        waveCount++;
    }

    lastValley = currentValue;
  }

  lastValue = currentValue;
}

void loop() {
  printValues();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // MPU reading window_slide filter
  MPU_total = MPU_total - MPU_readings[MPU_readIndex];
  MPU_readings[MPU_readIndex] = a.acceleration.z;
  MPU_total = MPU_total + MPU_readings[MPU_readIndex];
  MPU_readIndex = MPU_readIndex + 1;
  if (MPU_readIndex >= MPU_window_length) {
    MPU_readIndex = 0;
  }
  // calculate the average:
  MPU_average = MPU_total / MPU_window_length;
  Serial.println(MPU_average);
  countWavePeriod(MPU_average);

  if (waveCount / 2 > 5 && waveCount / 2 < 10)
    digitalWrite(MOTOR_PIN, HIGH);
  else
    digitalWrite(MOTOR_PIN, LOW);
  // digitalWrite(MOTOR_PIN, HIGH);

  delay(50);
}
