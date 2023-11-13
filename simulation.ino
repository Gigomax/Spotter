//Code for ELEC 390 Team 1 Simulations

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

unsigned long sendDataPrevMillis;
unsigned long checkBoolPrevMillis;

bool startRead;
bool stopRead;
bool calib;

LSM9DS1 imu;  // Create an LSM9DS1 object for each sensor
LSM9DS1 imu2;

// SDO_XM and SDO_G are both pulled high, for sensor 1
#define LSM9DS1_M  0x1E
#define LSM9DS1_AG 0x6B

// SDO_XM and SDO_G are both grounded, for sensor 2
#define LSM9DS1_M_2  0x1C
#define LSM9DS1_AG_2 0x6A

//Global variables for counts of reads
unsigned long startTime;
unsigned int accelReadCounter;
unsigned int gyroReadCounter;

unsigned int accelReadCounter2;
unsigned int gyroReadCounter2;

unsigned int lastaccelReadCounter;
unsigned int lastgyroReadCounter;

unsigned int lastaccelReadCounter2;
unsigned int lastgyroReadCounter2;

//Global variable for errors handling
float erroraccelx;
float erroraccely;
float erroraccelz;
float errorgyrox;
float errorgyroy;
float errorgyroz;

float erroraccelx2;
float erroraccely2;
float erroraccelz2;
float errorgyrox2;
float errorgyroy2;
float errorgyroz2;

bool checkerroraccel;
bool checkerroraccel2;

//Global variables to store sensor values
float accelx;
float accely;
float accelz;
float gyrox;
float gyroy;
float gyroz;

float accelx2;
float accely2;
float accelz2;
float gyrox2;
float gyroy2;
float gyroz2;

//Global variables fro gyro calculations
unsigned long gyrotime;
float anglex;
float angley;
float anglez;

unsigned long gyrotime2;
float anglex2;
float angley2;
float anglez2;

//Global variables for flex sensor setup
const int FLEX_PIN = A0; // Pin connected to voltage divider output
const float VCC = 3.3; // 3.3V Arduino Vref
const float R_DIV = 43000.0; // resistance of the voltage divider
//Adjusted Values by bending (trial&error with voltage divider of 43k)
const float STRAIGHT_VOLTAGE = 2.75; // tested at 0 deg
const float BEND_VOLTAGE = 2.39; // tested at 90 deg

//Global variables for flex sensor reads
int flexADC;
float voltageRead;
float flexAngle;

//Global variables for print rate
unsigned long lastPrint = 0;
unsigned long lastPrint2 = 0;
unsigned long lastPrint3 = 0;
const unsigned int PRINT_RATE = 500;

void setupGyro()
{
  imu.settings.gyro.enabled = true;  // Enable the gyro
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps for precision
  imu.settings.gyro.sampleRate = 3; // setting 3 means rate of 119Hz
  imu.settings.gyro.bandwidth = 0; //cutoff frequency depending on sample rate
  imu.settings.gyro.lowPowerEnable = false;
  imu.settings.gyro.HPFEnable = true; // HPF enabled
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  imu.settings.gyro.flipX = false;
  imu.settings.gyro.flipY = false;
  imu.settings.gyro.flipZ = false;
}

void setupAccel()
{
  imu.settings.accel.enabled = true; // Enable accelerometer and all axis...
  imu.settings.accel.enableX = true;
  imu.settings.accel.enableY = true;
  imu.settings.accel.enableZ = true;
  imu.settings.accel.scale = 8; // Set accel scale to +/-8g for precision
  imu.settings.accel.sampleRate = 1; //setting 1 is 10Hz.
  imu.settings.accel.bandwidth = 0; //setting 0 is for BW = 408Hz
  imu.settings.accel.highResEnable = false; // Disable HR
  imu.settings.accel.highResBandwidth = 0;  //setting 0 means ODR/50 (depends on sample rate)
}

uint16_t initLSM9DS1()
{
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters  
  return imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire); //sets up addres for accel/gyro & mag sensors
}

void setupGyro2()
{
  imu2.settings.gyro.enabled = true;  // Enable the gyro
  imu2.settings.gyro.scale = 245; // Set scale to +/-245dps for precision
  imu2.settings.gyro.sampleRate = 3; // setting 3 means rate of 119Hz
  imu2.settings.gyro.bandwidth = 0; //cutoff frequency depending on sample rate
  imu2.settings.gyro.lowPowerEnable = false;
  imu2.settings.gyro.HPFEnable = true; // HPF enabled
  imu2.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  imu2.settings.gyro.flipX = false;
  imu2.settings.gyro.flipY = false;
  imu2.settings.gyro.flipZ = false;
}

void setupAccel2()
{
  imu2.settings.accel.enabled = true; // Enable accelerometer and all axis...
  imu2.settings.accel.enableX = true;
  imu2.settings.accel.enableY = true;
  imu2.settings.accel.enableZ = true;
  imu2.settings.accel.scale = 8; // Set accel scale to +/-8g for precision
  imu2.settings.accel.sampleRate = 1; //setting 1 is 10Hz.
  imu2.settings.accel.bandwidth = 0; //setting 0 is for BW = 408Hz
  imu2.settings.accel.highResEnable = false; // Disable HR
  imu2.settings.accel.highResBandwidth = 0;  //setting 0 means ODR/50 (depends on sample rate)
}

uint16_t initLSM9DS1_2()
{
  setupGyro2(); // Set up gyroscope parameters
  setupAccel2(); // Set up accelerometer parameters
  return imu2.begin(LSM9DS1_AG_2, LSM9DS1_M_2, Wire); //sets up addres for accel/gyro & mag sensor
}

void calibration()
{
    while (!checkerroraccel) {
    if (imu.accelAvailable())
    {
      imu.readAccel();
      accelReadCounter++;
    }

    //imu 1 accel errors
    if (accelReadCounter != lastaccelReadCounter && accelReadCounter <= 100) {
      erroraccelx += imu.calcAccel(imu.ax);
      erroraccely += imu.calcAccel(imu.ay);
      erroraccelz += imu.calcAccel(imu.az) - 1;
      lastaccelReadCounter = accelReadCounter;
    } else if (accelReadCounter == 101) {
      erroraccelx /= 100;
      erroraccely /= 100;
      erroraccelz /= 100;
      checkerroraccel = true;
    }
  }

  errorgyrox = -0.121; //Direct Errors better than first 500 samples
  errorgyroy = 2.199;
  errorgyroz = 0.856;

  while (!checkerroraccel2) {
    if (imu2.accelAvailable())
    {
      imu2.readAccel();
      accelReadCounter2++;
    }

    //imu 2 accel errors
    if (accelReadCounter2 != lastaccelReadCounter2 && accelReadCounter2 <= 100) {
      erroraccelx2 += imu2.calcAccel(imu2.ax);
      erroraccely2 += imu2.calcAccel(imu2.ay);
      erroraccelz2 += imu2.calcAccel(imu2.az) - 1;
      lastaccelReadCounter2 = accelReadCounter2;
    } else if (accelReadCounter2 == 101) {
      erroraccelx2 /= 100;
      erroraccely2 /= 100;
      erroraccelz2 /= 100;
      checkerroraccel2 = true;
    }
  }

  errorgyrox2 = 0.300; //Direct Errors better than first 500 samples
  errorgyroy2 = 1.047;
  errorgyroz2 = -1.357;
}

void variablesInit ()
{
  accelReadCounter = 0;
  gyroReadCounter = 0;

  accelReadCounter2 = 0;
  gyroReadCounter2 = 0;

  erroraccelx = 0;
  erroraccely = 0;
  erroraccelz = 0;
  errorgyrox = 0;
  errorgyroy = 0;
  errorgyroz = 0;

  erroraccelx2 = 0;
  erroraccely2 = 0;
  erroraccelz2 = 0;
  errorgyrox2 = 0;
  errorgyroy2 = 0;
  errorgyroz2 = 0;

  checkerroraccel = false;
  checkerroraccel2 = false;

  gyrotime = -1;
  gyrotime2 = -1;

  startRead = false;
  stopRead = false;
  calib = false;

  sendDataPrevMillis = 0;
  checkBoolPrevMillis = 0;
}


void setup() {
  Serial.begin(115200);
  while(!Serial); //IMPORTANT TO WAIT UNTIL SERIAL COM ESTABLISHED
  delay(500);

  Wire.begin();
  
  uint16_t status = initLSM9DS1();
  uint16_t status_2 = initLSM9DS1_2();

  pinMode(FLEX_PIN, INPUT);

  variablesInit();

  startTime = millis();

  calibration();
}

void loop() {
  //For simulation read data from all sensors continuously (does not run with firebase)
  readSensor();
  readSensor2();
  readFlex();

  //Keep only prints required depending on simulation
  //each , separates two variables to have them in adjacent cells in excel
  //prinln is used for a new line to enter a new set of data
  //simulation is ran while monitored on excel data streamer
  
  Serial.print(anglex);
  Serial.print(", ");
  Serial.print(angley);
  Serial.print(", ");
  Serial.print(anglex2);
  Serial.print(", ");
  Serial.print(angley2);

  Serial.print(", ");
  Serial.print(accelx);
  Serial.print(", ");
  Serial.print(accely);
  Serial.print(", ");
  Serial.print(accelz);
  Serial.print(", ");
  Serial.print(accelx2);
  Serial.print(", ");
  Serial.print(accely2);
  Serial.print(", ");
  Serial.print(accelz2);

  Serial.print(", ");
  Serial.print(flexAngle);
  // Serial.print(", ");
  // Serial.print(gyroy);
  // Serial.print(", ");
  // Serial.print(gyrox2);
  // Serial.print(", ");
  // Serial.print(gyroy2);

  Serial.println();

  delay(100); //read every 0.1s to give enough time for data to be sent to excel data streamer
}

void readSensor()
{
  //SENSOR 1
  if (imu.accelAvailable())
  {
    imu.readAccel();
    accelx = imu.calcAccel(imu.ax) - erroraccelx;
    accely = imu.calcAccel(imu.ay) - erroraccely;
    accelz = imu.calcAccel(imu.az) - erroraccelz;
    accelReadCounter++;

    anglex = atan2(accely, accelz) * (180.0 / PI);
    angley = atan2(-accelx, sqrt(accely * accely + accelz * accelz)) * (180.0 / PI);
  }

  if (imu.gyroAvailable())
  {
    imu.readGyro();
    gyrox = imu.calcGyro(imu.gx) - errorgyrox;
    gyroy = imu.calcGyro(imu.gy) - errorgyroy;
    gyroz = imu.calcGyro(imu.gz) - errorgyroz;
    gyroReadCounter++;
    
    if (gyrotime != -1) {
      float time_diff = (millis() - gyrotime) * 0.001; //scale to get seconds
      //anglex += gyrox * time_diff; //gyro angle commented because not good precision
      //angley += gyroy * time_diff;
      //anglez += gyroz * time_diff;
    }
    
    gyrotime = millis();
  }
}

void readSensor2()
{
  //SENSOR 2
  if (imu2.accelAvailable())
  {
    imu2.readAccel();
    accelx2 = imu2.calcAccel(imu2.ax) - erroraccelx2;
    accely2 = imu2.calcAccel(imu2.ay) - erroraccely2;
    accelz2 = imu2.calcAccel(imu2.az) - erroraccelz2;
    accelReadCounter2++;

    anglex2 = atan2(accely2, accelz2) * (180.0 / PI);
    angley2 = atan2(-accelx2, sqrt(accely2 * accely2 + accelz2 * accelz2)) * (180.0 / PI);
  }

  if (imu2.gyroAvailable())
  {
    imu2.readGyro();
    gyrox2 = imu2.calcGyro(imu2.gx) - errorgyrox2;
    gyroy2 = imu2.calcGyro(imu2.gy) - errorgyroy2;
    gyroz2 = imu2.calcGyro(imu2.gz) - errorgyroz2;
    gyroReadCounter2++;

    if (gyrotime2 != -1) {
    float time_diff = (millis() - gyrotime2) * 0.001; //scale to get seconds
    //anglex2 += gyrox2 * time_diff; //gyro angle commented because not good precision
    //angley2 += gyroy2 * time_diff;
    //anglez2 += gyroz2 * time_diff;
    }
    
    gyrotime2 = millis();
  }
}

void readFlex()
{
  flexADC = analogRead(FLEX_PIN);
  voltageRead = flexADC * VCC / 1023;
  flexAngle = - 250 * voltageRead + 687.5; //found using manual computation of line
}