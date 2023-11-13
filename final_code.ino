//Final Code for ELEC 390 Team 1 Demo

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>

//For wifi-firebase connection
#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h> //Provide the token generation process info
#include <addons/RTDBHelper.h> //Provide the RTDB payload printing info and other helper functions

extern "C" {
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "c_types.h"
}

//Concordia wifi variables
char ssid[] = "ConcordiaUniversity";
char username[] = "netname"; //fill with info
char identity[] = "netname"; //fill with info
char password[] = "password"; //fill with info
uint8_t target_esp_mac[6] = {0x24, 0x0a, 0xc4, 0x9a, 0x58, 0x28};

//Define the WiFi credentials
#define WIFI_SSID "name"  //fill with info
#define WIFI_PASSWORD "password" //fill with info

//Firebase variables
#define API_KEY "AIzaSyCIwnjZL6hI299BDtL99JB_4XpSJv_4QOI"
#define DATABASE_URL "https://spotter-e1b60-default-rtdb.firebaseio.com"
#define USER_EMAIL "admins@email.com" //email and password of app sign in
#define USER_PASSWORD "adminpass"

FirebaseData fbdo; //Define Firebase Data object
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis;
unsigned long checkBoolPrevMillis;

//Communication between 
bool startRead;
bool stopRead;
bool calib;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
WiFiMulti multi;
#endif

//Create an LSM9DS1 object for each sensor
LSM9DS1 imu;
LSM9DS1 imu2;

// SDO_XM and SDO_G are both pulled high, for sensor 1 (done automatically - no wire needed)
#define LSM9DS1_M  0x1E
#define LSM9DS1_AG 0x6B

// SDO_XM and SDO_G are both grounded, for sensor 2 (done by connecting pins to GND with wire)
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

//Global variables for accel calculations
float angle_x;
float angle_y;

float angle_x2;
float angle_y2;

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

void calibration() //must  be done with sensors flat down
{
    while (!checkerroraccel) {
    if (imu.accelAvailable())
    {
      imu.readAccel();
      accelReadCounter++;
    }

    //imu 1 accel errors based on 100 values
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

  errorgyrox = -0.121; //Direct Errors from experimental data
  errorgyroy = 2.199;
  errorgyroz = 0.856;

  Serial.println("IMU 1 errors calculated and given by:");
  Serial.print("Accel x error: ");
  Serial.println(erroraccelx);
  Serial.print("Accel y error: ");
  Serial.println(erroraccely);
  Serial.print("Accel z error: ");
  Serial.println(erroraccelz);
  Serial.print("Gyro x error: ");
  Serial.println(errorgyrox);
  Serial.print("Gyro y error: ");
  Serial.println(errorgyroy);
  Serial.print("Gyro z error: ");
  Serial.println(errorgyroz);

  while (!checkerroraccel2) {
    if (imu2.accelAvailable())
    {
      imu2.readAccel();
      accelReadCounter2++;
    }

    //imu 2 accel errors based on 100 values
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

  errorgyrox2 = 0.300; //Direct Errors from experimental data
  errorgyroy2 = 1.047;
  errorgyroz2 = -1.357;

  Serial.println("IMU 2 errors calculated and given by:");
  Serial.print("Accel x error: ");
  Serial.println(erroraccelx2);
  Serial.print("Accel y error: ");
  Serial.println(erroraccely2);
  Serial.print("Accel z error: ");
  Serial.println(erroraccelz2);
  Serial.print("Gyro x error: ");
  Serial.println(errorgyrox2);
  Serial.print("Gyro y error: ");
  Serial.println(errorgyroy2);
  Serial.print("Gyro z error: ");
  Serial.println(errorgyroz2);

  Serial.println("Calibration is done!");
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

void setupWifiHome()
{
  #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  multi.addAP(WIFI_SSID, WIFI_PASSWORD);
  multi.run();
  #else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  #endif

  Serial.print("Connecting to Wi-Fi");
  unsigned long ms = millis();
  while (WiFi.status() != WL_CONNECTED) //show when trying to connnect
  {
    Serial.print(".");
    delay(300);
  #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    if (millis() - ms > 10000) //too long to connect
      break;
  #endif
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void setupWifiUni()
{
  Serial.print("Connecting to Wi-Fi");

  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  delay(1000);
  Serial.setDebugOutput(true);
  Serial.printf("SDK version: %s\n", system_get_sdk_version());
  Serial.printf("Free Heap: %4d\n",ESP.getFreeHeap());
  
  //Setting ESP into STATION mode
  wifi_set_opmode(STATION_MODE);

  struct station_config wifi_config;

  memset(&wifi_config, 0, sizeof(wifi_config));
  strcpy((char*)wifi_config.ssid, ssid);
  strcpy((char*)wifi_config.password, password);

  wifi_station_set_config(&wifi_config);
  wifi_set_macaddr(STATION_IF,target_esp_mac);
  

  wifi_station_set_wpa2_enterprise_auth(1);

  //Clear to remove any old data still inside
  wifi_station_clear_cert_key();
  wifi_station_clear_enterprise_ca_cert();
  wifi_station_clear_enterprise_identity();
  wifi_station_clear_enterprise_username();
  wifi_station_clear_enterprise_password();
  wifi_station_clear_enterprise_new_password();
  
  wifi_station_set_enterprise_identity((uint8*)identity, strlen(identity));
  wifi_station_set_enterprise_username((uint8*)username, strlen(username));
  wifi_station_set_enterprise_password((uint8*)password, strlen((char*)password));

  
  wifi_station_connect();
  while (WiFi.status() != WL_CONNECTED) { //show when trying to connect
    delay(1000);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void setupFirebase()
{
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback; //comes from TokenHelper.h

  #if defined(ESP8266)
    //In ESP8266 required buffer size for large data handle (can be increased)
    fbdo.setBSSLBufferSize(2048, 2048); //Rx buffer size & Tx buffer size
  #endif

  //Limit the size of response from FirebaseData
  fbdo.setResponseSize(2048);
  Firebase.begin(&config, &auth);

  #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    config.wifi.clearAP();
    config.wifi.addAP(WIFI_SSID, WIFI_PASSWORD);
  #endif

  Firebase.reconnectWiFi(true);
  Firebase.setDoubleDigits(5);
  config.timeout.serverResponse = 10 * 1000;
}

void setup() 
{
  Serial.begin(115200);
  while(!Serial); //IMPORTANT TO WAIT UNTIL SERIAL COM ESTABLISHED -> to see all prints
  delay(500);

  //run one of the two wifi setups depening on situation
  setupWifiUni(); //public WPA2 enterprise wifi with ssid, identity and password
  //setupWifiHome(); //private wifi with ssid and password
  
  setupFirebase();

  Wire.begin();
  
  Serial.println("Initializing the LSM9DS1");
  uint16_t status = initLSM9DS1();

  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX); //this prints confirms the imu is seen by arduino
  Serial.println("Should be 0x683D");
  Serial.println();

  
  Serial.println("Initializing the LSM9DS1 2");
  uint16_t status_2 = initLSM9DS1_2();

  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  Serial.println(status_2, HEX); //this prints confirms the imu is seen by arduino
  Serial.println("Should be 0x683D");
  Serial.println();

  pinMode(FLEX_PIN, INPUT); //setup for flex sensor

  variablesInit();

  //set all bools in firebase to false to avoid starting in an undefined state or reading too early
  Firebase.RTDB.setBool(&fbdo, F("/Flags/startRead"), startRead);  
  Firebase.RTDB.setBool(&fbdo, F("/Flags/stopRead"), stopRead);
  Firebase.RTDB.setBool(&fbdo, F("/Flags/calib"), calib);

  startTime = millis();
}

void loop() 
{
  if (Firebase.ready() && (millis() - checkBoolPrevMillis > 1000 || checkBoolPrevMillis == 0)) //delay to ensure polling is not done too rapidly
  { //get bools from firebase to determine next task to be executed
    Firebase.RTDB.getBool(&fbdo, F("/Flags/startRead"), &startRead);
    Firebase.RTDB.getBool(&fbdo, F("/Flags/calib"), &calib);
    checkBoolPrevMillis = millis();
  }

  if (calib) //run calibration
  {
    calibration();
    calib = false;
    Firebase.RTDB.setBool(&fbdo, F("/Flags/calib"), calib); //aknowledge to tell app it is done calibrating
  }

  if (startRead) //read sensor values and send to firebase
  {
    startRead = false;
    Firebase.RTDB.setBool(&fbdo, F("/Flags/startRead"), startRead); //acknoledge to tell app it is now starting to read
    
    do
    {
      readSensor(); //imu below knee read
      readSensor2(); //imu above knee read
      readFlex(); //flex sensor on back read
      sendFirebase(); //send values from all 3 sensors to firebase
      
      if (Firebase.ready() && (millis() - checkBoolPrevMillis > 1000 || checkBoolPrevMillis == 0)) //delay to ensure polling is not done too rapidly
      { //get bools from firebase to determine if it must stop reading
        Firebase.RTDB.getBool(&fbdo, F("/Flags/stopRead"), &stopRead);
        checkBoolPrevMillis = millis();
      }
    }
    while(!stopRead);
    
    stopRead = false;
    Firebase.RTDB.setBool(&fbdo, F("/Flags/stopRead"),stopRead); //acknowledge to tell app it stopped reading
  }
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
  
	//Angles from trigonometry
    angle_x = atan2(accely, accelz) * (180.0 / PI); //Bending angle of knee
    angle_y = atan2(-accelx, sqrt(accely * accely + accelz * accelz)) * (180.0 / PI);
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
      anglex += gyrox * time_diff; //angles not used
      angley += gyroy * time_diff;
      anglez += gyroz * time_diff;
    }
    
    gyrotime = millis();
  }

  if ((lastPrint + PRINT_RATE) < millis())
  {
    printSensorReadings();
    lastPrint = millis(); //NOTE: this number resets after 50 days (should not be an issue)
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

	//Angles from trigonometry
    angle_x2 = atan2(accely2, accelz2) * (180.0 / PI); //Bending angle of knee
    angle_y2 = atan2(-accelx2, sqrt(accely2 * accely2 + accelz2 * accelz2)) * (180.0 / PI);
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
    anglex2 += gyrox2 * time_diff; //angles not used
    angley2 += gyroy2 * time_diff;
    anglez2 += gyroz2 * time_diff;
    }
    
    gyrotime2 = millis();
  }
  

  if ((lastPrint2 + PRINT_RATE) < millis())
  {
    printSensorReadings2();
    lastPrint2 = millis();
  }
}

void readFlex()
{
  flexADC = analogRead(FLEX_PIN);
  voltageRead = flexADC * VCC / 1023; //reads 1023 for maximal voltage of 3.3V
  flexAngle = - 250 * voltageRead + 687.5; //found experimentall using manual computation of straight line equation
  
  if ((lastPrint3 + PRINT_RATE) < millis())
  {
    printFlexReading();
    lastPrint3 = millis();
  }
}

void sendFirebase()
{
  //Sending data to firebase every 0.1s
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 100 || sendDataPrevMillis == 0))
  {
    Firebase.RTDB.setFloat(&fbdo, F("/Sensor/Angle1x"), angle_x);
    Firebase.RTDB.setFloat(&fbdo, F("/Sensor/Angle1y"), angle_y);
    Firebase.RTDB.setFloat(&fbdo, F("/Sensor/Angle2x"), angle_x2);
    Firebase.RTDB.setFloat(&fbdo, F("/Sensor/Angle2y"), angle_y2);
    Firebase.RTDB.setFloat(&fbdo, F("/Sensor/Flex"), flexAngle);
    sendDataPrevMillis = millis();
  }
}

void printSensorReadings()
{
  //Printing only data sent to firebase for debugging (other prints can be activated if needed)
  //float runTime = (float)(millis() - startTime) / 1000.0;
  //float accelRate = (float)accelReadCounter / runTime;
  //float gyroRate = (float)gyroReadCounter / runTime;
  Serial.println("Sensor 1 data:");
  // Serial.print("A: ");
  // Serial.print(accelx);
  // Serial.print(", ");
  // Serial.print(accely);
  // Serial.print(", ");
  // Serial.print(accelz);
  // Serial.print(" g \t| ");
  // Serial.print(accelRate);
  // Serial.println(" Hz");
  // Serial.print("G: ");
  // Serial.print(anglex);
  // Serial.print(", ");
  // Serial.print(angley);
  // Serial.print(", ");
  // Serial.print(anglez);
  // Serial.print(" deg \t| ");
  // Serial.print(gyroRate);
  // Serial.println(" Hz");

  //Serial.print("Method 2: ");
  Serial.print("x axis: ");
  Serial.print(angle_x);
  Serial.print(", y axis: ");
  Serial.print(angle_y);
  Serial.println(" deg");

  Serial.println();
}

void printSensorReadings2()
{
  //Printing only data sent to firebase for debugging (other prints can be activated if needed)
  //float runTime = (float)(millis() - startTime) / 1000.0;
  //float accelRate = (float)accelReadCounter2 / runTime;
  //float gyroRate = (float)gyroReadCounter2 / runTime;
  Serial.println("Sensor 2 data:");
  // Serial.print("A: ");
  // Serial.print(accelx2);
  // Serial.print(", ");
  // Serial.print(accely2);
  // Serial.print(", ");
  // Serial.print(accelz2);
  // Serial.print(" g \t| ");
  // Serial.print(accelRate);
  // Serial.println(" Hz");
  // Serial.print("G: ");
  // Serial.print(anglex2);
  // Serial.print(", ");
  // Serial.print(angley2);
  // Serial.print(", ");
  // Serial.print(anglez2);
  // Serial.print(" deg \t| ");
  // Serial.print(gyroRate);
  // Serial.println(" Hz");

  //Serial.print("Method 2: ");
  Serial.print("x axis: ");
  Serial.print(angle_x2);
  Serial.print(", y axis");
  Serial.print(angle_y2);
  Serial.println(" deg");

  Serial.println();
}

void printFlexReading()
{
  //Printing angle of flex sensor
  Serial.println("Flex Sensor data:");
  
  Serial.print("angle: ");
  Serial.print(flexAngle);
  Serial.println(" deg");

  Serial.println();
}