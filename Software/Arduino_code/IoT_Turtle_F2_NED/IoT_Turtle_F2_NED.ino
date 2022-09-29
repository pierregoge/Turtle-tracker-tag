
/* IOT Turtle tracker software

   CMWX1ZZABZ (STM32L082 and SX1276)
   LSM303AGR
   MX25R6435FZAI 8 MByte SPI NOR flash memory

   Pierre Gogendeau (IOT Project)

   Inspired of Kris Winer code and librairies
*/

#include <STM32L0.h>
#include <RTC.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "GNSS.h"
#include "CayenneLPP.h"
#include "LSM303AGR_Acc.h"
#include "LSM303AGR_Mag.h"
#include "SPIFlash.h"
#include "I2CDev.h"
#include <CircularBuffer.h>
#include "MS5837.h"

// **** LoRAWan **** //

// IOTv3
/*const char *devEui = "70B3D57ED0052826";
  const char *appEui = "70B3D57ED00526F9";
  const char *appKey = "8B61780B700878ECC1A84BF2661CBD8C";*/

// IOT-024
const char *devEui = "383434305E37840A";
const char *appEui = "0000000000000000";
//const char *appEui = "1111111111111111";
const char *appKey = "D2B84D432BA764D193F72243960C23C9";


volatile bool serial_print = true;
volatile bool serial_debug = false;
volatile bool enable_gps_live = false;
volatile bool led_sampling = true;
// Sensor activation
volatile bool use_GPS  = true ;
volatile bool activate_rest  = true ; // usefull for debug and see if orientation if correct

// Size payload
const int payload = 220;
// Length max dive in second
const int len = 900; //sizeof(X1);
uint16_t time_out_GPS = 60000;  // ms (30sec)
const int nb_pos_out_max = 23;
const int nb_eth_out_max = 10;
int nb_pos_out = 0;
int nb_eth_out = 0;
long time_depth_offset = 600000; // 10min in millisecond

/*********************************************************************************/

#define PI 3.1415926
#define deg2rad(angle) (angle * PI / 180.0)
#define rad2deg(angle) (angle * 180.0 / PI)


#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use
I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// pin assignments
#define myLed     10 // blue led 
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

uint32_t UID[3] = {0, 0, 0};
char buffer[32];

uint8_t hours[4] = {12, 12, 12, 12}, minutes[4] = {0, 0, 0, 0}, seconds[4] = {0, 0, 0, 0}, year = 1, month = 1, day = 1;
uint32_t subSeconds[4], milliseconds;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp, battery_level;

//LSM303AGR_Acc definitions
#define LSM303AGR_Acc_intPin1 A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_Acc_intPin2 3   // interrupt2 pin definitions, significant motion

/*union {             // scale resolutions per LSB for the accel sensor
  float fval;
  byte bval[4];
  } aRes;*/


union {           // offset biases
  float fval;
  byte bval[4];
} accelBias[3];


float aRes;
float param_accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accelData[3], accTempData;  // Stores the 10-bit signed accel output if normal mode
float ax, ay, az;                   // variables to hold latest accel data values
uint8_t Ascale = AFS_2G, AODR_rec;
uint8_t AODR = 0x02;
volatile bool newLSM303AGR_AccData_1 = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_AccData_2 = false; // used for motion ready interrupt handling
bool newAcc = false;
LSM303AGR_Acc LSM303AGR_Acc(&i2c_0); // instantiate LSM303AGR_Acc class

//LSM303AGR_Mag definitions
#define LSM303AGR_Mag_intPin  A2 // interrupt for magnetometer data ready
int16_t magData[3];
uint8_t MODR_rec; // = MODR_10Hz;
uint8_t MODR = 0x00;
float mx, my, mz;                   // variables to hold latest accel data values

union {           // offset biases
  float fval;
  byte bval[4];
} magBias[3];

float mRes = 0.0015f;            // mag sensitivity
volatile bool newLSM303AGR_MagData = false; // used for data ready interrupt handling
bool newMag = false;
float param_magBias[3] = {0.0f, 0.0f, 0.0f}, param_magScale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
LSM303AGR_Mag LSM303AGR_Mag(&i2c_0); // instantiate LSM303AGR_Mag class

// **** GPS **** //

volatile bool isTracking = false;

GNSSLocation myLocation;
GNSSSatellites mySatellites;

#define GNSS_en      5     // enable for GNSS 3.0 V LDO
#define pps          4     // 1 Hz fix pulse
#define GNSS_backup A0     // RTC backup for MAX M8Q

uint8_t GPS_Hour = 12, GPS_Minute = 0, GPS_Second = 0, GPS_Year = 1, GPS_Month = 1, GPS_Day = 1;
//uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
//uint32_t subSeconds, milliseconds;
bool ppsFlag = false, firstSync = false, alarmFlag = true;
uint16_t count = 0, fixType = 0, fixQuality, latBytes[4], longBytes[4], tempBytes[4], pressBytes[4];
int32_t latOut, longOut;
float Long, Lat, Alt, EPE;
float EHPE = 99.9f ;
int Nb_Satellite ;
bool InMotion = true;
bool new_pos_gps = false;


static const char *fixTypeString[] = { "NONE", "TIME", "2D", "3D", };

static const char *fixQualityString[] = { "", "", "/DIFFERENTIAL", "/PRECISE", "/RTK_FIXED", "/RTK_FLOAT", "/ESTIMATED", "/MANUAL", "/SIMULATION", };

float lon_out[nb_pos_out_max], lat_out[nb_pos_out_max];

//TimerMillis Timer_GPS ;

CayenneLPP myLPP(payload) ;
//TimerMillis LoRaTimer ;


// **** Variable computing **** //

// Low-pass filter : Moving average filter
float sum_buf_x = 0;
float sum_buf_y = 0;
float sum_buf_z = 0;
float agx_b = 0;   // calculate the moving average
float agy_b = 0;   // calculate the moving average
float agz_b = 0;   // calculate the moving average
float sum_buf_mx = 0;
float sum_buf_my = 0;
float sum_buf_mz = 0;
float mgx_b = 0;   // calculate the moving average
float mgy_b = 0;   // calculate the moving average
float mgz_b = 0;   // calculate the moving average


// Buffer to process data
const int len_buffer = 3;
//Raw accel
CircularBuffer<float, len_buffer> buf_ax;
CircularBuffer<float, len_buffer> buf_ay;
CircularBuffer<float, len_buffer> buf_az;
//Raw Mag
CircularBuffer<float, len_buffer> buf_mx;
CircularBuffer<float, len_buffer> buf_my;
CircularBuffer<float, len_buffer> buf_mz;

//Gravity Accel
CircularBuffer<float, len_buffer> buf_agx;
CircularBuffer<float, len_buffer> buf_agy;
CircularBuffer<float, len_buffer> buf_agz;
//Gravity Mag
CircularBuffer<float, len_buffer> buf_mgx;
CircularBuffer<float, len_buffer> buf_mgy;
CircularBuffer<float, len_buffer> buf_mgz;
//Dynamic Accel
CircularBuffer<float, len_buffer> buf_adx;
CircularBuffer<float, len_buffer> buf_ady;
CircularBuffer<float, len_buffer> buf_adz;

float acc_gx, acc_gy, acc_gz;
float acc_dx, acc_dy, acc_dz;

float q[3];
float pitch, roll, yaw;
float pitch2, roll2, yaw2;
float heading, heading2;
float depth;
float h_speed;
float VeDBA;
float decli = +19.6 / 180 * PI;
float d_t = 0;
float tortuisity = 0;

uint16_t  inc_dive = 1;
uint16_t  inc_e = 0;
uint16_t  inc = 1;
uint16_t inc_filter = 1;

// Data to process before sending
int16_t buf_1[len] = {0};
int16_t buf_2[len] = {0};
int16_t z[len] = {0};
int16_t heading_array[len] = {0};
//int16_t speed_h[len] = {0};
int16_t offset_z = 0;
int16_t min_z = 0;

float x_out[nb_pos_out_max];
float y_out[nb_pos_out_max];
float z_out[nb_pos_out_max];


int16_t etho[nb_eth_out_max][4] = {0};
int16_t inc_eth = 0;
int16_t  etho_buf[3] = {0};
int inc_heading = 0;


//Slope variable calculation
int depth_slope;

//const int len_traj_out = 10;
//CircularBuffer<uint16_t, len_traj_out> x_out;
//CircularBuffer<uint16_t, len_traj_out> y_out;

// Variables algo speed 2
float fixed_speed = 0.45;
float p_lim = 0.3490; // in rad (20°)

const int nb_pts_max = 10;



uint16_t ind_min[1];


// MS5837 configuration
// Specify sensor full scale
uint8_t MS5837_OSR = ADC_8192;     // set pressure amd temperature oversample rate

uint16_t MS5837_Pcal[8];         // calibration constants from MS5837 PROM registers
unsigned char MS5837_nCRC;       // calculated check sum to ensure PROM integrity
uint32_t MS5837_D1 = 0, MS5837_D2 = 0;  // raw MS5837 pressure and temperature data
double MS5837_dT, MS5837_OFFSET, MS5837_SENS, MS5837_TT2, MS5837_OFFSET2, MS5837_SENS2;  // First order and second order corrections for raw MS5837 temperature and pressure data

double MS5837_Temperature, MS5837_Pressure; // stores MS5837 pressures sensor pressure and temperature
float fluidDensity = 1029.0f; // kg/m^3 for seawater

MS5837 MS5837(&i2c_0); // instantiate MS5837 class


// Variable and flag for ethogram
int flag_e[5][2] = {0};
int flag_s[5][2] = {0};

const int short_dive_limit = 30;
const int slope_up = -100;
const int slope_down = 100;
const int VeDBA_rest = 10;
const int surf_depth = 10;

// Timer to start a behavior (u = up, d = down, r = rest, s = swim, surf = surface)
const int t_d2u = 15;
const int t_d2r = 15;
const int t_r2u = 10;
const int t_r2d = 15;
const int t_u2surf = 15;
const int t_s2r = 15;
const int t_s2u = 15;
const int t_s2d = 15;

// Time to end a behavior to go in swim phase
const int t_d2s = 15;
const int t_u2s = 15;
const int t_r2s = 15;
const int t_surf2s = 15;

// Algorithm flag
bool start_dive_flag = 1;
bool end_dive_flag = 0;
bool flag_chg_behavior = 0;



// Function andrea

void GPS_first_fix( bool enable_serialPrint_data_GPS );
void read_update_GPS_data( bool enable_serialPrint_data_GPS );
void send_loRa_data( bool enable_serialPrint_loRaWan) ;
void init_loRa( void );
void read_battery_level( bool enable_serialPrint_battery );


// Timer
volatile bool flag_sampling_during_GPS = false; // used for data ready interrupt handling
TimerMillis samplingDuringGPS;  // instantiate low-frequency timer
volatile bool flag_new_depth_offset = false; // used for data ready interrupt handling
TimerMillis samplingDepthOffset;  // instantiate low-frequency timer



//**************************************************************************************************************************
// TEST

/************************************************************
  Strcut Point for the cartesian coordiante before LAT, LON conversion
************************************************************/

struct Point {
  float x;
  float y;
  float z;
};

Point VectorPoint[nb_pos_out_max];

int NextVectorPoint = 0;

int addPoint(float x, float y, float z) // (3 bit color 1-red 2-green 4-blue)
{
  VectorPoint[NextVectorPoint].x = x;
  VectorPoint[NextVectorPoint].y = y;
  VectorPoint[NextVectorPoint].z = z;
  //VectorPoint[NextVectorPoint].color = color;
  return NextVectorPoint++;
}

void dropPoint()
{
  for (int t = 0; t < NextVectorPoint; t++)
  {
    VectorPoint[t].z--;
  }
}

float earthRadius = 6367; //radius in km


//*************** DEBUG *************************//

float    moy_heading = 0;
float   sum_heading = 0;
float    nb_heading = 0;

//************************************************************************************************************************************


void setup()
{


  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)

  // Set the RTC time to firmware build time
  //SetDefaultRTC();

  Serial.begin(115200);
  delay(5000);
  Serial.println("\r\n--------------------------------------------------------------------");
  Serial.print("-----------------    TEST TRAJ    04/2022    ----------------\r\n");
  Serial.println("--------------------------------------------------------------------\r\n");

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);

  // Info about LoRa
  LoRaWAN.getDevEui(buffer, 18);                      // Get DevEUI
  Serial.print("DevEUI \t "); Serial.println(buffer); // Display DevEUI
  Serial.print("AppKey \t "); Serial.println(appKey); // Display Appkey
  Serial.print("AppEUI \t "); Serial.println(appEui); // Display AppEUI
  Serial.println();

  // Init LoRa
  LoRaWAN.begin(EU868);
  LoRaWAN.setADR(false);
  LoRaWAN.setDataRate(4); // 0 => SF = 12 | 1 => SF = 11 | 2 => SF 10 ... Careful with the size of the payload
  LoRaWAN.setTxPower(0);
  LoRaWAN.setSubBand(1); // 1 for MTCAP, 2 for TT gateways
  LoRaWAN.joinOTAA(appEui, appKey, devEui);


  // Batttery pin setup
  pinMode(myVBat_en, OUTPUT);
  digitalWrite(myVBat_en, LOW); // start with battery voltage monirtor off
  pinMode(myVBat, INPUT);
  analogReadResolution(12);

  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  STM32L0Temp = STM32L0.getTemperature();

  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("STM32L0 MCU Temperature = "); Serial.print(STM32L0Temp, 2); Serial.print(" C\r\n");
  Serial.println(" ");

  switch (AODR)
  {
    case 0x01:
      AODR_rec = 1;
      break;
    case 0x02:
      AODR_rec = 10;
      break;
    case 0x04:
      AODR_rec = 50;
      break;
    case 0x05:
      AODR_rec = 100;
      break;
  }

  switch (MODR)
  {
    case 0x01:
      MODR_rec = 10;
      break;
    case 0x02:
      MODR_rec = 10;
      break;
    case 0x04:
      MODR_rec = 50;
      break;
    case 0x05:
      MODR_rec = 100;
      break;
  }


  printAscale();
  printAODR();

  //init I2C
  I2C_BUS.begin();                                      // Set master mode
  delay(1000);
  I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
  delay(1000);

  // set up Accelero
  pinMode(LSM303AGR_Acc_intPin1, INPUT);

  digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures


  aRes = LSM303AGR_Acc.getAres(Ascale); // get sensor resolution, only need to do this once
  LSM303AGR_Acc.selfTest();
  LSM303AGR_Acc.reset();
  LSM303AGR_Acc.init(Ascale, AODR);
  LSM303AGR_Acc.offsetBias(param_accelBias);
  /*param_accelBias[0] = -0.028;
    param_accelBias[1] = -0.074;
    param_accelBias[2] = -0.010;*/
  Serial.println("accel biases (mg)"); Serial.println(1000.0f * param_accelBias[0]); Serial.println(1000.0f * param_accelBias[1]); Serial.println(1000.0f * param_accelBias[2]);
  delay(1000);

  digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)

  // set up Magnéto
  //pinMode(LSM303AGR_Mag_intPin, OUTPUT);
  LSM303AGR_Mag.selfTest();
  LSM303AGR_Mag.reset();
  LSM303AGR_Mag.init(AODR);
  LSM303AGR_Mag.offsetBias(param_magBias, param_magScale);
  /* param_magBias[0] = 0.003;
    param_magBias[1] = 0.021;
    param_magBias[2] = 0.006;
    param_magScale[0] = 1;//0.94;
    param_magScale[1] = 1;//1.07;
    param_magScale[2] = 1;*/
  /*[0] = -0.547;
    param_magBias[1] = 0.018;
    param_magBias[2] = 0.459;
    param_magScale[0] = 0.97;
    param_magScale[1] = 1.02;
    param_magScale[2] = 1.01;*/
  Serial.println("mag biases (mG)"); Serial.println(1000.0f * param_magBias[0]); Serial.println(1000.0f * param_magBias[1]); Serial.println(1000.0f * param_magBias[2]);
  Serial.println("mag scale (mG)"); Serial.println(param_magScale[0]); Serial.println(param_magScale[1]); Serial.println(param_magScale[2]);
  delay(2000); // add delay to see results before serial spew of data

  delay(100); //stabilisation mesures

  Serial.println("\r\nDataLog running...");


  //attachInterrupt(LSM303AGR_Acc_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR_Acc
  //attachInterrupt(LSM303AGR_Mag_intPin, myinthandler2, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR_Acc

  LSM303AGR_Acc.readAccData(accelData); // read data register to clear interrupt before main loop
  LSM303AGR_Mag.readMagData(magData); // read data register to clear status register

  getRTC(0); //timestamp

  // Init pressure
  // Reset the MS5837 pressure sensor
  MS5837.Reset();
  delay(100);
  Serial.println("MS5837 pressure sensor reset...");
  // Read PROM data from MS5837 pressure sensor
  MS5837.PromRead(MS5837_Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(MS5837_Pcal[0]);
  unsigned char MS5837_refCRC = MS5837_Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(MS5837_Pcal[1]);
  Serial.print("C2 = "); Serial.println(MS5837_Pcal[2]);
  Serial.print("C3 = "); Serial.println(MS5837_Pcal[3]);
  Serial.print("C4 = "); Serial.println(MS5837_Pcal[4]);
  Serial.print("C5 = "); Serial.println(MS5837_Pcal[5]);
  Serial.print("C6 = "); Serial.println(MS5837_Pcal[6]);

  MS5837_nCRC = MS5837.checkCRC(MS5837_Pcal);  //calculate checksum to ensure integrity of MS5837 calibration data
  Serial.print("Checksum = "); Serial.print(MS5837_nCRC); Serial.print(" , should be "); Serial.println(MS5837_refCRC);


  // First depth data to setup offset


  // Pressure MS5837 Data
  MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
  MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
  MS5837_dT = MS5837_D2 - MS5837_Pcal[5] * pow(2, 8); // calculate temperature difference from reference
  MS5837_OFFSET = MS5837_Pcal[2] * pow(2, 16) + MS5837_dT * MS5837_Pcal[4] / pow(2, 7);
  MS5837_SENS = MS5837_Pcal[1] * pow(2, 15) + MS5837_dT * MS5837_Pcal[3] / pow(2, 8);

  MS5837_Temperature = (2000 + (MS5837_dT * MS5837_Pcal[6]) / pow(2, 23)) / 100;     // First-order Temperature in degrees Centigrade

  // Second order corrections
  if (MS5837_Temperature > 20)
  {
    MS5837_TT2 = 2 * MS5837_dT * MS5837_dT / pow(2, 37); // correction for high temperatures
    MS5837_OFFSET2 = 1 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 16;
    MS5837_SENS2 = 0;
  }
  if (MS5837_Temperature < 20)                  // correction for low temperature
  {
    MS5837_TT2      = 3 * MS5837_dT * MS5837_dT / pow(2, 33);
    MS5837_OFFSET2 = 3 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 2;
    MS5837_SENS2   = 5 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 8;
  }
  if (MS5837_Temperature < -15)                     // correction for very low temperature
  {
    MS5837_OFFSET2 = MS5837_OFFSET2 + 7 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
    MS5837_SENS2 = MS5837_SENS2 + 4 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
  }
  // End of second order corrections

  MS5837_Temperature = MS5837_Temperature - MS5837_TT2 / 100;
  MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
  MS5837_SENS = MS5837_SENS - MS5837_SENS2;

  MS5837_Pressure = (((MS5837_D1 * MS5837_SENS) / pow(2, 21) - MS5837_OFFSET) / pow(2, 13)) / 10; // Pressure in mbar or hPa

  // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
  float MS5837_depth = (MS5837_Pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);
  offset_z = int16_t(MS5837_depth * 100);

  if (serial_print) {
    Serial.print("Depth :"); Serial.println(MS5837_depth);
    Serial.print("offset_z :"); Serial.println(offset_z);
  }



  pinMode(GNSS_backup, OUTPUT);       // Power for MAX M8Q RTC backup
  digitalWrite(GNSS_backup, HIGH);    // Setup GNSS

  // --- Configuration of GNSS --- //
  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ);        // Start GNSS
  while (GNSS.busy()) { }                                     // Wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS) ; // Choose satellites
  while (GNSS.busy()) { }                                     // Wait for set to complete

  GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL);                     // GNSS.ANTENNA_INTERNAL or GNSS.ANTENNA_EXTERNAL
  while (GNSS.busy()) { }                                     // Wait for set to complete

  GNSS.enableWakeup();                                        // Wake up
  while (GNSS.busy()) { }                                     // Wait for set to complete

  //Timer_GPS.start( Read_GPS , 0 , 20000 );
  if ( use_GPS == true ) {
    GPS_first_fix( true ) ;
  } else {
    Lat = -21.170064; //-21.102168; 
    //Lat = -21.042492; //-21.094209; //Eperon, Lat = -20.932937; // Le Port  // planch al Lat=-21.094209
    Long = 55.287344; // EperonLong =  55.290304; // Le Port // planch al Long= 55.234457;
    lat_out[0] = Lat;
    lon_out[0] = Long;
  }

  GNSS.suspend() ;
  new_pos_gps = true;


  //init_loRa();

  delay(1000);
  digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
  //pinMode(myLed, INPUT);

  buf_1[0] = 0;
  buf_2[0] = 0;

  //Timer sampling during GPS
  samplingDuringGPS.start(callbackSamplingDuringGPS, 0,   1000);    // high freq (one minute) timer
  //Timer offset pressure sensor
  samplingDepthOffset.start(callbackDepthOffset, 0,   time_depth_offset);    // high freq (one minute) timer

  STM32L0.stop();        // Enter STOP mode and wait for an interrupt





} /* end of setup */

//******************************************************************************************************************************************
/*

   Everything in the main loop is based on interrupts, so that
   if there has not been an interrupt event the STM32L082 should be in STOP mode
*/

/*unsigned long t_start;
  unsigned long t_end;
  unsigned long t_filter;
  unsigned long t_traj;
  unsigned long t_lora;*/

int  inc_timing_traj = 0;
int  inc_timing_filter = 0;

void loop()
{

  //Serial.println("loop");

  if (start_dive_flag == 1) {

    digitalWrite(myLed, LOW); // Turn on led when New Dive or fixed successfully completed
    start_dive_flag = 0;
    buf_1[0] = 0;
    buf_2[0] = 0;
    inc = 1;
    d_t = 0;

    if (serial_print) {
      Serial.println("New Dive");
    }

    inc_eth = 0;
    etho[inc_eth][1] = 5;
    etho[inc_eth][2] = inc;
    etho[inc_eth][4] = z[inc];

    //GPS
    if ( use_GPS == true ) {
      read_update_GPS_data( true );
      inc++;

      /*etho[inc_eth][3] = inc-1; // End of the behavior
        inc_eth++;
        etho[inc_eth][1] = 1;                 // Start SWIM behavior
        etho[inc_eth][2] = inc;   // Start time of the new behavior
        etho[inc_eth][4] = z[inc];
        flag_chg_behavior = 1;*/

    }


    NextVectorPoint = 0;
    if (new_pos_gps == true) {
      lat_out[0] = Lat;
      lon_out[0] = Long;
      //lla2ecef(Lat, Long,0);
      new_pos_gps = false;
    } else {
      lat_out[0] = lat_out[nb_pos_out - 1];
      lon_out[0] = lon_out[nb_pos_out - 1];
      //lla2ecef(lat_out[nb_pos_out - 1], lon_out[nb_pos_out - 1],0);
    }

    digitalWrite(myLed, HIGH); // turn off led when GPS Timeout or fixed successfully completed

  }


  if (flag_chg_behavior == 1) {


    flag_chg_behavior = 0;
    int first_data = etho[inc_eth - 1][2];
    int last_data =  etho[inc_eth - 1][3];
    speed_fct_1(etho[inc_eth - 1][1]); // speed calculation with fct 1
    htrack_fct(first_data, last_data, serial_debug);

    if (serial_print) {
      Serial.println("New Behavior");
      Serial.println("Start speed and traj calculation");
      Serial.print("first_data : "); Serial.println(first_data);
      Serial.print("last_data :"); Serial.println(last_data);
      Serial.print("Speed : ");  Serial.println(h_speed);
    }

    inc_heading = 0;

  }

  if (flag_sampling_during_GPS == true)
  {

    if (led_sampling) {
      digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
    }


    flag_sampling_during_GPS = false;

    //if (newLSM303AGR_AccData_1 == true) // on interrupt, read data
    //{
    //digitalWrite(myLed, HIGH);    // turn off led
    // newLSM303AGR_AccData_1 = false;     // reset newData flag

    newAcc = true;
    LSM303AGR_Acc.readAccData(accelData); // INT1 cleared on any read
    /*Serial.print("acc data : ");  Serial.println(accelData[0]);
      Serial.print("ARes: ");  Serial.println(aRes);
      Serial.print("accelBias : ");  Serial.println(param_accelBias[0]);*/
    //if (inc <= len_buffer)
    //{
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelData[0] * aRes - param_accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)accelData[1] * aRes - param_accelBias[1];
    az = (float)accelData[2] * aRes - param_accelBias[2];

    /*Serial.print("ax : ");  Serial.println(ax); // Test timing
      Serial.print("ay : ");  Serial.println(ay); // Test timing
      Serial.print("az : ");  Serial.println(az); // Test timing*/

    // Fill the buffer of raw accel data

    //buf_acc_x.unshift(ax);
    //Serial.print("buf_acc_x : ");  Serial.println(buf_acc_x[0]); // Test timing
    //buf_acc_y.unshift(ay);
    //buf_acc_z.unshift(az);


    //digitalWrite(myLed, HIGH);    // turn off led
    //newLSM303AGR_MagData = false;     // reset newData flag

    byte c = 0;
    //Serial.print("mag 1  ");
    //delay(1);
    while (!((c >> 3) & 0x01))  // attente data mag
    {
      c = LSM303AGR_Mag.getStatus();
    }

    LSM303AGR_Mag.readMagData(magData);  // INT2 cleared on any read

    newMag = true;
    //if (inc <= len_buffer)
    //{

    //Serial.print("mag data : ");  Serial.println(magData[0]);
    //Serial.print("mRes: ");  Serial.println(mRes);
    //Serial.print("magBias : ");  Serial.println(param_magScale[0]);

    mx = (float)magData[0] * mRes - param_magBias[0]; // get actual G value
    my = (float)magData[1] * mRes - param_magBias[1];
    mz = (float)magData[2] * mRes - param_magBias[2];
    mx *= param_magScale[0];
    my *= param_magScale[1];
    mz *= param_magScale[2];


    // Pressure MS5837 Data
    MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
    MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
    MS5837_dT = MS5837_D2 - MS5837_Pcal[5] * pow(2, 8); // calculate temperature difference from reference
    MS5837_OFFSET = MS5837_Pcal[2] * pow(2, 16) + MS5837_dT * MS5837_Pcal[4] / pow(2, 7);
    MS5837_SENS = MS5837_Pcal[1] * pow(2, 15) + MS5837_dT * MS5837_Pcal[3] / pow(2, 8);

    MS5837_Temperature = (2000 + (MS5837_dT * MS5837_Pcal[6]) / pow(2, 23)) / 100;     // First-order Temperature in degrees Centigrade

    // Second order corrections
    if (MS5837_Temperature > 20)
    {
      MS5837_TT2 = 2 * MS5837_dT * MS5837_dT / pow(2, 37); // correction for high temperatures
      MS5837_OFFSET2 = 1 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 16;
      MS5837_SENS2 = 0;
    }
    if (MS5837_Temperature < 20)                  // correction for low temperature
    {
      MS5837_TT2      = 3 * MS5837_dT * MS5837_dT / pow(2, 33);
      MS5837_OFFSET2 = 3 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 2;
      MS5837_SENS2   = 5 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 8;
    }
    if (MS5837_Temperature < -15)                     // correction for very low temperature
    {
      MS5837_OFFSET2 = MS5837_OFFSET2 + 7 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
      MS5837_SENS2 = MS5837_SENS2 + 4 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
    }
    // End of second order corrections

    MS5837_Temperature = MS5837_Temperature - MS5837_TT2 / 100;
    MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
    MS5837_SENS = MS5837_SENS - MS5837_SENS2;

    MS5837_Pressure = (((MS5837_D1 * MS5837_SENS) / pow(2, 21) - MS5837_OFFSET) / pow(2, 13)) / 10; // Pressure in mbar or hPa

    // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
    float MS5837_depth = (MS5837_Pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);

    /*if (flag_new_depth_offset == true){
      offset_z = min_z;
      min_z = 1000;
      flag_new_depth_offset = false;
            if (serial_debug){
        Serial.print("offset_z :"); Serial.println(offset_z);
      }
      }*/


    z[inc] = int16_t(MS5837_depth * 100) - offset_z;


    /*if (z[inc] < min_z){
      min_z = z[inc];
      if (serial_debug){
        Serial.print("min_z :"); Serial.println(min_z);
      }
      }

    */if (serial_print) {
      Serial.print("Depth :"); Serial.println(MS5837_depth);
      Serial.print("Depth_z :"); Serial.println(z[inc]);
    }
  }




  if (newMag == true && newAcc == true)
  {

    newMag = false;
    newAcc = false;


    //Serial.print("ax = "); Serial.print(int(1000*ax));
    //Serial.print(" ay = "); Serial.print(int(1000*ay));
    //Serial.print(" az = "); Serial.print(int(1000*az)); Serial.println(" mg");
    //Serial.print("mx = "); Serial.print(int(1000*mx));
    //Serial.print(" my = "); Serial.print(int(1000*my));
    //Serial.print(" mz = "); Serial.print(int(1000*mz)); Serial.println(" mG");

    /*int ax_buf = (1000*ax);
      int ay_buf = (1000*ay);
      int az_buf = (1000*az);*/

    if (inc_filter <= len_buffer)
    {
      sum_buf_x = sum_buf_x + ax;
      sum_buf_y = sum_buf_y + ay;
      sum_buf_z = sum_buf_z + az;

      sum_buf_mx = sum_buf_mx + mx;
      sum_buf_my = sum_buf_my + my;
      sum_buf_mz = sum_buf_mz + mz;

      buf_ax.unshift(ax);
      buf_ay.unshift(ay);
      buf_az.unshift(az);

      buf_mx.unshift(mx);
      buf_my.unshift(my);
      buf_mz.unshift(mz);

      //Serial.print("sum_buf_z = "); Serial.println(sum_buf_z);

      // Filter raw acc to get gravitationnal acceleration
      agx_b = sum_buf_x / (inc_filter);
      //Serial.print("acc_gx : ");  Serial.println(int16_t(acc_gx*10000)); // Test timing
      agy_b = sum_buf_y / (inc_filter);
      agz_b = sum_buf_z / (inc_filter);

      // Filter raw acc to get gravitationnal mag
      mgx_b = sum_buf_mx / (inc_filter);
      //Serial.print("acc_gx : ");  Serial.println(int16_t(acc_gx*10000)); // Test timing
      mgy_b = sum_buf_my / (inc_filter);
      mgz_b = sum_buf_mz / (inc_filter);

      //Serial.print("agz_b = "); Serial.println(agz_b);

    } else  //Length /2 of the moving average filter. Data is delayed of this number of sample
    {
      // Serial.print("inc : ");  Serial.println(inc);
      //float last_depth = depth;
      // Sample depth

      //t_start = micros();     // Test timing

      sum_buf_x = sum_buf_x + ax - buf_ax[len_buffer - 1];
      sum_buf_y = sum_buf_y + ay - buf_ay[len_buffer - 1];
      sum_buf_z = sum_buf_z + az - buf_az[len_buffer - 1];

      sum_buf_mx = sum_buf_mx + mx - buf_mx[len_buffer - 1];
      sum_buf_my = sum_buf_my + my - buf_my[len_buffer - 1];
      sum_buf_mz = sum_buf_mz + mz - buf_mz[len_buffer - 1];

      //Serial.print("sum_buf_z = "); Serial.print(sum_buf_mz);
      //Serial.print("buf_acc_z END : ");  Serial.println(buf_mz[len_buffer-1]); // Test timing
      // Low-pass filter : Moving average filter
      // Fill the buffer of raw accel data
      buf_ax.unshift(ax);
      buf_ay.unshift(ay);
      buf_az.unshift(az);

      buf_mx.unshift(mx);
      buf_my.unshift(my);
      buf_mz.unshift(mz);

      //Serial.print("buf_acc_z 0 : ");  Serial.println(buf_mz[0]); // Test timing
      //Serial.print("buf_acc_z END : ");  Serial.println(buf_mz[len_buffer-1]); // Test timing


      // Filter raw acc to get gravitationnal acceleration
      agx_b = sum_buf_x / (len_buffer);
      //Serial.print("acc_gx : ");  Serial.println(int16_t(acc_gx*10000)); // Test timing
      agy_b = sum_buf_y / (len_buffer);
      agz_b = sum_buf_z / (len_buffer);

      // Filter raw acc to get gravitationnal acceleration
      mgx_b = sum_buf_mx / (len_buffer);
      //Serial.print("acc_gx : ");  Serial.println(int16_t(acc_gx*10000)); // Test timing
      mgy_b = sum_buf_my / (len_buffer);
      mgz_b = sum_buf_mz / (len_buffer);

      //Serial.print("sum_buf_x = "); Serial.print(sum_buf_mx);
      //Serial.print("sum_buf_y = "); Serial.print(sum_buf_my);
      //Serial.print("sum_buf_z = "); Serial.print(sum_buf_mz); Serial.println(" mG");

    }

    //Serial.print("agx_b = "); Serial.print(mgx_b);
    //Serial.print("agy_b = "); Serial.print(mgy_b);
    //Serial.print("agz_b = "); Serial.print(mgz_b); Serial.println(" mg");

    float agx = (agy_b) * 1; //(agx_b)*1;//float(100);
    float agy = (agx_b) * -1; //;//float(100);
    float agz = agz_b;//float(100);

    float adx = (ay - agx);//float(100);
    float ady = (-ax - agy);//float(100);
    float adz = az - agz;//float(100);

    float mgx = (mgy_b) * -1; //float(100);
    float mgy = (mgx_b) * 1; //float(100);
    float mgz = (mgz_b) * -1; //float(100);


    /*t_end = micros();     // Test timing
      Serial.print("Time filter 1Hz: ");  // Test timing
      t_filter = t_filter + (t_end - t_start);
      inc_timing_filter++;
      Serial.println(t_end - t_start);  // Test timing*/

    Serial.print("agx = "); Serial.print(int(1000 * agx));
    Serial.print("agy = "); Serial.print(int(1000 * agy));
    Serial.print("agz = "); Serial.print(int(1000 * agz)); Serial.println(" mg");
    Serial.print("adx = "); Serial.print(int(1000 * adx));
    Serial.print("ady = "); Serial.print(int(1000 * ady));
    Serial.print("adz = "); Serial.print(int(1000 * adz)); Serial.println(" mg");

    /*pitch = pitch_fct(agy, agx, agz);
      roll = roll_fct(agx, agz);
      heading = yaw_fct(pitch, roll, mgy, mgx, mgz);*/

    // Freescale solution
    // Normal axis
    roll = atan2(agy, agz);
    pitch = atan(-agx / (agy * sin(roll) + agz * cos(roll)));
    float magn_fy_fs = mgz * sin(roll) - mgy * cos(roll);
    float magn_fx_fs = mgx * cos(pitch) + mgy * sin(pitch) * sin(roll) + mgz * sin(pitch) * cos(roll);

    //Correction axis
    /*roll = atan2(agx, agz);
      pitch = atan(-agy / (agx * sin(roll) + agz * cos(roll)));

      float magn_fy_fs = mgz * sin(roll) - mgx * cos(roll);
      float magn_fx_fs = mgy * cos(pitch) + mgx * sin(pitch) * sin(roll) + mgz * sin(pitch) * cos(roll);*/

    yaw = atan2(magn_fy_fs, magn_fx_fs);

    //yaw = atan2(my, mx);
    SAAM(agx, agy, agz, mgx, mgy, -mgz);

    yaw2   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch2 = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll2  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);



    //ToEulerAngles(q[0], q[1], q[2], q[3]);
    float heading_offset = (19.f / 180) * PI;
    Serial.print("heading_offset : ");    Serial.println(heading_offset); // Test timing
    heading = yaw - decli + heading_offset;// - (18/180)*PI;// - (90 / 180) * PI;
    heading2 = yaw2 - decli;// - (18/180)*PI; // - (90 / 180) * PI;

    if (heading < 0) {
      heading = heading + (2 * PI) ;
    }

    if (heading > 2 * PI) {
      heading = heading - (2 * PI);
    }

    if (heading2 < 0) {
      heading2 = heading2 + (2 * PI);
    }

    if (heading2 > 2 * PI) {
      heading2 = heading2 - (2 * PI);
    }

    //float heading_2 = atan2(mx, my);

    VeDBA = sqrt(pow(adx, 2) + pow(ady, 2) + pow(adz, 2));

    if (serial_print) {
      Serial.print("yaw : ");    Serial.println(float(heading * 180) / PI); // Test timing
      Serial.print("yaw 2 : ");  Serial.println(heading2 * 180 / PI); // Test timing
      Serial.print("pitch : ");  Serial.println(pitch * 180 / PI); // Test timing// Test timing
      //Serial.print("pitch 2 : ");  Serial.println(pitch2 * 180 / PI); // Test timing// Test timing
      Serial.print("roll : ");  Serial.println(roll * 180 / PI); // Test timing// Test timing
      // Serial.print("roll 2  : ");  Serial.println(roll2 * 180 / PI); // Test timing// Test timing
      Serial.print("VeDBA : ");  Serial.println(VeDBA * 1000); // Test timing// Test timing

      Serial.println("accel biases (mg)"); Serial.println(1000.0f * param_accelBias[0]); Serial.println(1000.0f * param_accelBias[1]); Serial.println(1000.0f * param_accelBias[2]);
      Serial.println("mag biases (mG)"); Serial.println(1000.0f * param_magBias[0]); Serial.println(1000.0f * param_magBias[1]); Serial.println(1000.0f * param_magBias[2]);
      Serial.println("mag scale (mG)"); Serial.println(param_magScale[0]); Serial.println(param_magScale[1]); Serial.println(param_magScale[2]);

    }

    inc_filter++;

    sum_heading = sum_heading + heading;
    nb_heading = nb_heading + 1;
    //heading_array[inc_heading] = int16_t(heading*100);
    heading_array[inc] = uint16_t(heading * 10000);

    //Serial.print("heading_array[inc] :"); Serial.println(heading_array[inc]);


    /*// Compute dynamic acceleration
      acc_dx = buf_acc_x[2] - acc_gx;
      //Serial.print("acc_dx : ");  Serial.println(int16_t(acc_dx*10000)); // Test timing
      acc_dy = buf_acc_y[2] - acc_gy;
      acc_dz = buf_acc_z[2] - acc_gz;*/


    // ALGO 1

    // Ethogram
    etho_1_fct(); // Ethogram calculation

    inc++; // Increment number of dive sample

    if (serial_print) {
      Serial.println("**************************************************************************************");
      Serial.print("Inc : "); Serial.println(inc);  // Test timing
      Serial.println("**************************************************************************************");
    }

    if (led_sampling) {
      digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
    }


  }


  //if (end_dive_flag == 1) { // New dive detected
  if (inc >= len - 1 || end_dive_flag == 1) { // End of dive detected

    samplingDuringGPS.stop();

    moy_heading = sum_heading / nb_heading;
    sum_heading = 0;
    nb_heading = 0;

    end_dive_flag = 0;

    // ADD in etho function
    start_dive_flag = 1;
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));

    // FIN ALGO 1 PARTI 2/2
    //Serial.println("End Dive");
    //Serial.println("Speed and traj calculation last section");

    int nb_eth = inc_eth + 1;

    int first_data = etho[inc_eth][2];
    int last_data =  inc;
    //Serial.print("first_data : "); Serial.println(first_data);
    //Serial.print("last_data"); Serial.println(last_data);

    /*heading = float(buf_1[ii]) / 100;
      pitch = float(buf_2[ii]) / 100; // Can be used in speed function*/
    //Serial.print("heading : ");  Serial.println(heading);
    //Serial.print("Pitch : ");  Serial.println(pitch);



    speed_fct_1(etho[inc_eth][1]);   // speed calculation with fct 1
    //Serial.print("Speed : ");  Serial.println(h_speed);

    htrack_fct(first_data, last_data, serial_debug );

    //Serial.print("inc eth: ");  Serial.println(nb_eth);

    //Compute message variable
    //Allocate ethogram payload size
    int payload_traj;

    if (enable_gps_live == true) {

      if (nb_eth < 10) {
        payload_traj = (payload - 15) - (8 * nb_eth);
      }

      nb_pos_out = payload_traj / 11; // Length GPS data

    } else {

      if (nb_eth < 10) {
        payload_traj = (payload - 15 - 11) - (8 * nb_eth);
      }

      nb_pos_out = payload_traj / 8;  // Length X,Y,Z data in cayenne

    }

    nb_eth_out = nb_eth;
    x_out[0] = 0;
    y_out[0] = 0;

    if (serial_debug) {

      Serial.print("inc eth: ");  Serial.println(nb_eth);
      Serial.print("payload_traj: ");  Serial.println(payload_traj);
      Serial.print("nb_eth_out: "); Serial.println(nb_eth_out);
      Serial.print("nb_pos_out: "); Serial.println(nb_pos_out);

      Serial.print("x_out[0] : "); Serial.println(x_out[0]);
      Serial.print("y_out[0] : "); Serial.println(y_out[0]);
      Serial.print("lat_out[0] : "); Serial.println(lat_out[0]);
      Serial.print("lon_out[0] : "); Serial.println(lon_out[0]);
    }

    //Reduce trajectory
    if (serial_print) {
      Serial.println("Compression V1");  // Test timing
    }

    //nb_pos_out = 10;

    int inc_comp = inc / (nb_pos_out - 1); // Remove 1rt and last
    //Serial.print("inc_comp : "); Serial.println(inc_comp);

    for (int i = 1; i < nb_pos_out - 1; i++) { //We reserve the last position for the real iteration of trajectory

      //Serial.print("inc_comp * i: "); Serial.println(inc_comp * i);

      x_out[i] = float(buf_1[inc_comp * i]) / 100;
      y_out[i] = float(buf_2[inc_comp * i]) / 100;

      float d = sqrt(  pow(x_out[i] -  x_out[i - 1], 2) + pow( y_out[i] - y_out[i - 1], 2));

      if (serial_print) {
        Serial.print("x_out 1: "); Serial.println(x_out[i]);
        Serial.print("y_out 1: "); Serial.println(y_out[i]);
        Serial.print("d : "); Serial.println(d);
      }

      //z_out[i] = z[inc_comp * i];

      int16_t ii = inc_comp * i;

      //Serial.print("x_out : "); Serial.println(x_out[i]);
      //Serial.print("y_out : "); Serial.println(y_out[i]);

      /*float s_h = float(speed_h[ii])/100;

        speed_h[ii] = 10;*/

      /*Serial.print("s_h : "); Serial.println(s_h);
        Serial.print("heading_array[ii] : "); Serial.println(heading_array[ii]);*/



      if (enable_gps_live == true) {

        float q = d / (earthRadius * 1000);
        float heading_f = atan2(y_out[i] - y_out[i - 1], x_out[i] -  x_out[i - 1]);

        lat_out[i] = asin(sin(deg2rad(lat_out[i - 1])) * cos(q) + cos(deg2rad(lat_out[i - 1])) * sin(q) * cos(heading_f));
        lon_out[i] = deg2rad(lon_out[i - 1]) + atan2(sin(heading_f) * sin(q) * cos(deg2rad(lat_out[i - 1])), (cos(q) - sin(deg2rad(lat_out[i - 1])) * sin(deg2rad(lat_out[i]))));

        lat_out[i]  = rad2deg(lat_out[i]);
        lon_out[i] = rad2deg(lon_out[i]);

        //Reduced trajectory
        if (serial_print) {
          Serial.print("lat1:"); Serial.println(lat_out[i], 9);
          Serial.print("lon1:"); Serial.println(lon_out[i], 9);
        }

      }


    }

    Serial.print("buf_1 last : "); Serial.println(buf_1[inc]);
    x_out[nb_pos_out - 1] = float(buf_1[inc]) / 100;
    y_out[nb_pos_out - 1] = float(buf_2[inc]) / 100;

    if (serial_print) {
      Serial.print("last x_out : "); Serial.println(x_out[nb_pos_out - 1]);
      Serial.print("last y_out  : "); Serial.println(y_out[nb_pos_out - 1]);
    }

    float d_end = sqrt(  pow( x_out[nb_pos_out - 1] -  x_out[0], 2) + pow( y_out[nb_pos_out - 1] - y_out[0], 2));   // Total distance between the strat en end of the dive
    tortuisity = d_t / d_end;
    //Serial.print("Tortuisity: "); Serial.println(tortuisity);
    //Serial.print("d_t: "); Serial.println(d_t);



    if (enable_gps_live == false) { // Calculate new GPS pos for next dive


      float q = d_end / (earthRadius * 1000);
      float heading_f = atan2(y_out[nb_pos_out - 1] - y_out[0], x_out[nb_pos_out - 1] -  x_out[0]) ;
      Serial.print("d_end: "); Serial.println(d_end);
      Serial.print("heading_f: "); Serial.println(heading_f);

      lat_out[nb_pos_out - 1] = asin(sin(deg2rad(lat_out[0])) * cos(q) + cos(deg2rad(lat_out[0])) * sin(q) * cos(heading_f));
      lon_out[nb_pos_out - 1] = deg2rad(lon_out[0]) + atan2(sin(heading_f) * sin(q) * cos(deg2rad(lat_out[0])), (cos(q) - sin(deg2rad(lat_out[0])) * sin(deg2rad(lat_out[nb_pos_out - 1]))));

      lat_out[nb_pos_out - 1]  = rad2deg(lat_out[nb_pos_out - 1]);
      lon_out[nb_pos_out - 1] = rad2deg(lon_out[nb_pos_out - 1]);

      //Reduce trajectory
      if (serial_print) {
        Serial.print("lat2:"); Serial.println(lat_out[nb_pos_out - 1], 9);
        Serial.print("lon2:"); Serial.println(lon_out[nb_pos_out - 1], 9);
      }

    } else {

      int last_i = nb_pos_out - 1;
      int b_last_i = nb_pos_out - 2;

      float d = sqrt(  pow(x_out[last_i] -  x_out[b_last_i], 2) + pow( y_out[last_i] - y_out[b_last_i], 2));
      float q = d / (earthRadius * 1000);
      float heading_f = atan2(y_out[last_i] - y_out[b_last_i], x_out[last_i] -  x_out[b_last_i]);

      lat_out[last_i] = asin(sin(deg2rad(lat_out[b_last_i])) * cos(q) + cos(deg2rad(lat_out[b_last_i])) * sin(q) * cos(heading_f));
      lon_out[last_i] = deg2rad(lon_out[b_last_i]) + atan2(sin(heading_f) * sin(q) * cos(deg2rad(lat_out[b_last_i])), (cos(q) - sin(deg2rad(lat_out[b_last_i])) * sin(deg2rad(lat_out[last_i]))));

      lat_out[last_i]  = rad2deg(lat_out[last_i]);
      lon_out[last_i] = rad2deg(lon_out[last_i]);

      //Reduced trajectory
      if (serial_print) {
        Serial.print("lat:"); Serial.println(lat_out[last_i], 9);
        Serial.print("lon:"); Serial.println(lon_out[last_i], 9);
      }



    }




    //Read Vbat
    read_battery_level( serial_print ) ;


    //Store data and send message
    send_loRa_data( serial_print ) ;
    delay(1000);
    samplingDuringGPS.start(callbackSamplingDuringGPS, 0,   1000);    // high freq (one minute) timer

    //Clear variable
    buf_mx.clear();
    buf_my.clear();
    buf_mz.clear();

    buf_ax.clear();
    buf_ay.clear();
    buf_az.clear();

    sum_buf_x = 0;
    sum_buf_y = 0;
    sum_buf_z = 0;

    sum_buf_mx = 0;
    sum_buf_my = 0;
    sum_buf_mz = 0;

    inc_dive++;
    inc_filter = 1;

    inc_e = 0;

    buf_1[0] = 0;
    buf_2[0] = 0;

  }

  STM32L0.stop();        // Enter STOP mode and wait for an interrupt



}  /* end of loop*/

//***************************************************************************************************************************

void myinthandler1()
{
  newLSM303AGR_AccData_1 = true;
  STM32L0.wakeup();
  //Serial.println("int wake up accel");
}

void callbackSamplingDuringGPS(void)
{
  flag_sampling_during_GPS = true;
  //Serial.println("callbackSamplingDuringGPS");
  STM32L0.wakeup();
}

void callbackDepthOffset(void) {
  flag_new_depth_offset = true;
  STM32L0.wakeup();
}


void SAAM(float axb, float ayb, float azb, float mxb, float myb, float mzb)
{
  float mD = axb * mxb + ayb * myb + azb * mzb;
  float mN = sqrt(1 - mD * mD);
  float norm_q;

  q[0] = - ayb * (mN + mxb) + axb * myb;
  q[1] =  (azb - 1) * (mN + mxb) + axb * (mD - mzb);
  q[2] =  (azb - 1) * myb + ayb * (mD - mzb);
  q[3] =  azb * mD - axb * mN - mzb;

  norm_q = sqrt(pow(q[0], 2) + pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2));

  for (int i = 0; i < 4; i++) {
    q[i] = q[i] / norm_q;
  }

}

void ToEulerAngles(float qw, float qx, float qy, float qz) {



  // roll (x-axis rotation)
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  roll2 = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = 2 * (qw * qy - qz * qx);
  /*if (std::abs(sinp) >= 1)
      angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else*/
  pitch2 = asin(sinp);

  // yaw (z-axis rotation)
  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  yaw2 = atan2(siny_cosp, cosy_cosp);

}


Point convertSphericalToCartesian(float latitude, float longitude)
{
  latitude = latitude / 180 * 3.14159;
  longitude = longitude / 180 * 3.14159;
  float z = earthRadius * sin(latitude);
  float x = earthRadius * cos(latitude) * cos(longitude);
  float y = earthRadius * cos(latitude) * sin(longitude);

  addPoint(x, y, z);
}

Point convertCartesianToSpherical(float x_b, float y_b, float z_b, int inc_b)
{


  float r = sqrt(x_b * x_b + y_b * y_b + z_b * z_b);
  lat_out[inc_b] = asin(z_b / r) * 180 / 3.14159;
  lon_out[inc_b] = atan2(y_b, x_b) * 180 / 3.14159;

}


/*void ecef2lla(float x_b, float y_b, float z_b, int inc_b)
  {


    float x_sqrd = pow(x_b, 2);
    float y_sqrd = pow(y_b, 2);
    float z_sqrd = pow(z_b, 2);

    float lon = atan2(y_b, x_b);

    float p      = sqrt(x_sqrd + y_sqrd);
    float p_sqrd = pow(p, 2);
    float F      = 54 * b_sqrd * z_sqrd;
    Serial.print("F:"); Serial.println(F);
    float G      = p_sqrd + ((1 - ecc_sqrd) * z_sqrd) - (ecc_sqrd * (a_sqrd - b_sqrd));
    float G_sqrd = pow(G, 2);
    float c      = (pow(ecc, 4) * F * p_sqrd) / pow(G, 3);
    float c_sqrd = pow(c, 2);
    float s      = pow(1 + c + sqrt(c_sqrd + (2 * c)), 1.5);
    float k      = s + 1 + (1 / s);
    float k_sqrd = pow(k, 2);
    float P      = F / (3 * k_sqrd * G_sqrd);
    Serial.print("P:"); Serial.println(P);
    float Q      = sqrt(1 + (2 * pow(ecc, 4) * P));
    Serial.print("Q:"); Serial.println(Q);
    float r0     = ((-P * ecc_sqrd * p) / (1 + Q)) + sqrt((0.5 * a_sqrd * (1 + (1 / Q))) - ((P * (1 - ecc_sqrd) * z_sqrd) / (Q * (1 + Q))) - (0.5 * P * p_sqrd));
    Serial.print("r0:"); Serial.println(r0);
    float U      = sqrt(pow(p - (ecc_sqrd * r0), 2) + z_sqrd);
    float V      = sqrt(pow(p - (ecc_sqrd * r0), 2) + ((1 - ecc_sqrd) * z_sqrd));
    Serial.print("V:"); Serial.println(V);
    float z0     = (b_sqrd * z_b) / (a * V);
    Serial.print("z0:"); Serial.println(z0);

    float h   = U * (1 - (b_sqrd / (a * V)));
    float lat = atan2(z_b + (ecc_prime_sqrd * z0), p);*/

/*   lat_out[inc_b] = asin(sin(deg2rad(lat_out[inc_b-1])) * cos(speed_h/(earthRadius*1000)) + cos(deg2rad(lat_out[inc_b-1])) * sin(speed_h[ii]/(earthRadius*1000)) * cos(heading_array[inc]));
   lon_out[inc_b] = lon_out[inc_b-1] + atan2(sin(heading_array[ii])*sin(speed/(earthRadius*1000))*cos(deg2rad(lat_out[inc_b-1])),(cos(speed_h[ii]/(earthRadius*1000)) - sin(deg2rad(lat_out[inc_b-1])) * sin(deg2rad(lat_out[inc_b]))));

   lat_out[inc_b]  = rad2deg(lat_out[inc_b]);
   lon_out[inc_b] = rad2deg(lon_out[inc_b]);

  }

  void lla2ecef(float lat, float lon,   float alt)
  {

  lat = deg2rad(lat);
  lon = deg2rad(lon);

  float x = earthRadius * cos(lat) * cos(lon);
  float y = earthRadius * cos(lat) * sin(lon);
  float z = ((1 - ecc_sqrd) * earthRadius) * sin(lat);

  addPoint(x, y, z);
  }*/


/*void myinthandler2()
  {
  Serial.println("int2");
  newLSM303AGR_MagData = true;
  STM32L0.wakeup();
  }*/

float pitch_fct(float xx, float yy, float zz)
{
  float pitch = atan2(-xx, sqrt(pow(yy, 2) + pow(zz, 2)));
  return pitch;
}


float roll_fct(float yy, float zz)
{
  float roll = atan2(yy, zz);
  return roll;
}

float yaw_fct(float f_pitch, float f_roll, float f_mx, float f_my, float f_mz)
{
  float by2 = f_mz * sin(f_roll) - f_my * cos(f_roll);
  float bz2 = f_my * sin(f_roll) + f_mz * cos(f_roll);
  float bx3 = f_mx * cos(f_pitch) + bz2 * sin(f_pitch);
  float yaw = atan2(by2, bx3);
  return yaw;
}


void etho_1_fct()
{
  if (serial_print) {
    Serial.print("Behavior :");  // Test timing
    Serial.println(etho[inc_eth][1]);  // Test timing
  }
  // Starting FLag section


  int nb_sample_end;

  //REST : Test if ODBA inferior to a threshold.
  if (VeDBA * 1000 < VeDBA_rest)
  {

    Serial.print("flag_s[2][1] :");  // Test timing
    Serial.println(flag_s[2][1]);  // Test timing
    flag_s[2][1]++;
    flag_s[2][2] = inc;
  } else {
    flag_s[2][1] = 0;
  }


  // Starting FLag section
  //REST : Test if ODBA superior to a threshold
  if ((VeDBA * 1000 >= VeDBA_rest))
  {
    Serial.print("flag_e[2][1] :");  // Test timing
    Serial.println(flag_e[2][1]);  // Test timing

    flag_e[2][1]++;
    flag_e[2][2] = inc;
  } else {
    flag_e[2][1] = 0;
  }


  // Changing behavior section

  // SURFACE -> SWIMMING
  if (etho[inc_eth][1] == 5)
  { // If surface)

    if (z[inc] < surf_depth)  // If depth > 1m
    {
      //nb_sample_end = flag_e[2][2] - flag_e[2][1];
      etho[inc_eth][3] = inc - 1;
      inc_eth++;
      etho[inc_eth][1] = 1;  // swimming phase
      etho[inc_eth][2] = inc;
      etho[inc_eth][4] = z[inc];
      flag_chg_behavior = 1;
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }



    /*// TO SWIM
      // if (z[inc] > 300)  // If depth > 1m
      if (flag_e[2][1] >= t_r2u) //SWIM : Compare flag number and sampling (in nb of sample)
      {
      nb_sample_end = flag_e[2][2] - flag_e[2][1];
      etho[inc_eth][3] = nb_sample_end - 1;
      inc_eth++;
      etho[inc_eth][1] = 1;  // swimming phase
      etho[inc_eth][2] = nb_sample_end;
      etho[inc_eth][4] = z[nb_sample_end];
      flag_chg_behavior = 1;
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
      }

      // TO REST
      if (flag_s[2][1] >= t_r2u) //REST : Compare flag number and sampling (in nb of sample)
      {
      nb_sample_end = flag_s[2][2] - flag_s[2][1];
      etho[inc_eth][3] = nb_sample_end - 1;
      inc_eth++;
      etho[inc_eth][1] = 2;  // Rest phase
      etho[inc_eth][2] = nb_sample_end;
      etho[inc_eth][4] = z[nb_sample_end];
      flag_chg_behavior = 1;
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
      }*/

  }


  //Rest phase -> SWIM
  if (etho[inc_eth][1] == 2)
  {

    digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
    delay(200); //stabilisation mesures
    digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)

    if (flag_e[2][1] >= t_r2u) //UP : Compare flag number and sampling (in nb of sample)
    {
      if (serial_debug)
      {
        Serial.print("flag_e[2][1] :"); Serial.println(flag_e[2][1]); // Test timing
        Serial.print("flag_e[2][2] :"); Serial.println(flag_e[2][2]); // Test timing
        Serial.print("nb_sample_end :"); Serial.println(nb_sample_end); // Test timing
      }
      nb_sample_end = flag_e[2][2] - flag_e[2][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 1;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      etho[inc_eth][4] = z[nb_sample_end];
      flag_chg_behavior = 1;
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }

    /*if (z[inc] < surf_depth)  // If depth > 1m
      {
      etho[inc_eth][3] = inc;
      start_dive_flag = 1;
      end_dive_flag = 1;
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
      }*/


  }

  //SWIM or UP phase -> END DIVE
  if (etho[inc_eth][1] == 1 ) { // If SWIM

    digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
    delay(100); //stabilisation mesures
    digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
    delay(100); //stabilisation mesures
    digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
    delay(100); //stabilisation mesures
    digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)

    if (z[inc] > surf_depth)  // If depth >  surf limit
    {
      etho[inc_eth][3] = inc;
      start_dive_flag = 1;
      end_dive_flag = 1;
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }

    // TO REST
    if (activate_rest == true) {
      if (flag_s[2][1] >= t_r2u) //UP : Compare flag number and sampling (in nb of sample)
      {
        if (serial_debug)
        {
          Serial.print("flag_e[2][1] :"); Serial.println(flag_e[2][1]); // Test timing
          Serial.print("flag_e[2][2] :"); Serial.println(flag_e[2][2]); // Test timing
          Serial.print("nb_sample_end :"); Serial.println(nb_sample_end); // Test timing
        }

        nb_sample_end = flag_s[2][2] - flag_s[2][1];
        etho[inc_eth][3] = inc - 1;
        inc_eth++;
        etho[inc_eth][1] = 2;  // Rest phase
        etho[inc_eth][2] = inc;
        etho[inc_eth][4] = z[inc];
        flag_chg_behavior = 1;
        memset(flag_s, 0, sizeof(flag_s));
        memset(flag_e, 0, sizeof(flag_e));
      }

    }
  }


}


/* LoRa */

void init_loRa( void ) {

  // --- Configuration LoRaWAN --- //
  // Asia AS923 | Australia  AU915 | Europe EU868 | India IN865 | Korea KR920 | US US915 (64 + 8 channels)

  LoRaWAN.begin(EU868);
  LoRaWAN.setADR(false);
  LoRaWAN.setDataRate(4); // 0 => SF = 12 | 1 => SF = 11 | 2 => SF 10 ... Careful with the size of the payload
  LoRaWAN.setTxPower(0);
  LoRaWAN.setSubBand(1); // 1 for MTCAP, 2 for TT gateways

  LoRaWAN.joinOTAA(appEui, appKey, devEui);

  // LoRa Timer 1min
  // LoRaTimer.start( Send_LoRa_Data , 0 , 30000 ) ;      //  1 minute period, delayed 1 minute
}

void send_loRa_data( bool enable_serialPrint_loRaWan) {

  // STM32L0.wakeup() ;


  // --- Send Data to LoRa --- //
  /*if ( enable_serialPrint_loRaWan == true ) {
    Serial.println( (String)"LoRaWAN: Busy   " + LoRaWAN.busy() )   ; // Display state of LoRa - 0 for false (= available) - 1 for true (= busy)
    Serial.println( (String)".        Joined " + LoRaWAN.joined() ) ; // Display LoRa connection - 0 for false (= not joined) - 1 for true (= joined)
    }*/


  if ( !LoRaWAN.busy() && LoRaWAN.joined() ) { // if LoRa available (not(0)=1=true) AND LoRa joined then

    Serial.print("moy_heading : "); Serial.println(moy_heading);
    myLPP.reset(); // Reset writing payload
    myLPP.addDigitalInput(3, inc_dive); // Dive Nb
    myLPP.addTemperature(4, d_t); // add distance travelled
    //myLPP.addTemperature(5, moy_heading); // add Battery_Level
    //myLPP.addAnalogInput(5, battery_level)          ; // add Battery_Level
    myLPP.addAnalogInput(5, tortuisity); // add tortuisity
    myLPP.addLuminosity(6, inc); // add length dive

    for (int ii = 0; ii < nb_eth_out; ii++) {

      myLPP.addAccelerometer(7, float(etho[ii][2]) / 100, etho[ii][1], float(etho[ii][4]) / 100) ; // Store strating time of ethogram, behavior and depth
    }

    if (enable_gps_live == true) {

      for (int ii = 0; ii < nb_pos_out; ii++) {

        // Adding conversion

        myLPP.addGPS(8, lat_out[ii], lon_out[ii], z_out[ii]) ;                ; // add GPS
      }

    } else {

      myLPP.addGPS(8, lat_out[0], lon_out[0], 0) ;                ; // add GPS

      for (int ii = 1; ii < nb_pos_out; ii++) {

        // Adding conversion

        myLPP.addAccelerometer(9, float(x_out[ii]) / 10, float(y_out[ii]) / 10, float(z_out[ii]) / 10) ; // Store strating time of ethogram, behavior and depth
      }


    }

    Serial.print("Message size : ") ; Serial.print(myLPP.getSize()); // Display a msg

    if (enable_gps_live == true) { // Calculate new GPS pos for next dive

      LoRaWAN.sendPacket(136, myLPP.getBuffer(), myLPP.getSize());

    } else {


      LoRaWAN.sendPacket(136, myLPP.getBuffer(), myLPP.getSize());

    }


    if ( enable_serialPrint_loRaWan == true ) {
      Serial.println(".        Msg send") ;  // Display a msg
    }

  } // if ( !LoRaWAN.busy() && LoRaWAN.joined() )
  else {
    if ( enable_serialPrint_loRaWan == true ) {
      Serial.println(".        Msg not send") ;  // Display a msg
    }
  }

  // STM32L0.stop( ) ;

} // void callbackLoRaTx(void)




/* GPS */

void GPS_first_fix( bool enable_serialPrint_data_GPS ) {

  Serial.println("GPS_First_Fix");

  STM32L0.wakeup() ;

  delay(100);

  GNSS.resume();
  while (GNSS.busy()) { }                                     // Wait for set to complete

  int now = millis() ;

  EHPE = 999.99f ;
  //Serial.println(EHPE);

  // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

  while ( (EHPE >= 10.0) ) { // Waiting to have a "good" EHPE

    //Serial.println(EHPE);
    // Serial.println( (String)"While loop - condition n°1 : " + (millis() - now <= 30000) ) ;
    // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

    // Serial.println("Hello");

    EHPE = 999.99f ;

    // --- GNSS --- //
    if (GNSS.location(myLocation) ) {

      if ( enable_serialPrint_data_GPS == true ) {
        Serial.print("LOCATION: ");
        Serial.print(fixTypeString[myLocation.fixType()]);
        if ( fixTypeString[myLocation.fixType()] == "NONE") {
          Serial.println(".") ;
        }
      }

      if (myLocation.fixType() != GNSSLocation::TYPE_NONE) {

        GPS_Year   = myLocation.year()    ;
        GPS_Month  = myLocation.month()   ;
        GPS_Day    = myLocation.day()     ;
        GPS_Hour   = myLocation.hours()   ;
        GPS_Minute = myLocation.minutes() ;
        GPS_Second = myLocation.seconds() ;

        // Display date
        if ( enable_serialPrint_data_GPS == true ) {
          Serial.print(fixQualityString[myLocation.fixQuality()]);
          Serial.print(" ");
          Serial.print(myLocation.year());
          Serial.print("/");
          Serial.print(myLocation.month());
          Serial.print("/");
          Serial.print(myLocation.day());
          Serial.print(" ");

          // Display hours
          if (myLocation.hours() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.hours());
          Serial.print(":");

          if (myLocation.minutes() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.minutes());
          Serial.print(":");

          if (myLocation.seconds() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.seconds());


          if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED) {
            Serial.print(" ");
            Serial.print(myLocation.leapSeconds());
            if (!myLocation.fullyResolved()) {
              Serial.print("D");
            }
          }

        }

        if (myLocation.fixType() != GNSSLocation::TYPE_TIME) {

          Lat = myLocation.latitude();
          myLocation.latitude(latOut);
          Long = myLocation.longitude();
          myLocation.longitude(longOut);
          Alt = myLocation.altitude();
          EHPE = myLocation.ehpe(); // use this as accuracy figure of merit
          Nb_Satellite = mySatellites.count() ;

          if ( enable_serialPrint_data_GPS == true ) {
            Serial.print(" : ");
            Serial.print(Lat, 7); Serial.print(","); Serial.print(Long, 7);
            Serial.print(",");
            Serial.print(Alt, 3);
            Serial.print(" EHPE :");
            Serial.print(EHPE, 3);
            Serial.print(",");
            Serial.print(myLocation.evpe(), 3);
            Serial.print(" SAT. :");
            Serial.print(myLocation.satellites());
            Serial.print(" DOP :");
            Serial.print(myLocation.hdop(), 2);
            Serial.print(",");
            Serial.print(myLocation.vdop(), 2);
            Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;

          }



          // if( (myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE < 10.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          // if( (EPE < 50.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          //   Serial.println("***GNSS go to sleep!***");
          //   GNSS.suspend(); // once we have a good 3D location fix put CAM M8Q to sleep
          //   //break ;
          // }

        } // if (myLocation.fixType() != GNSSLocation::TYPE_TIME)

      } // if (myLocation.fixType() != GNSSLocation::TYPE_NONE)

    } // if (GNSS.location(myLocation))


    if (GNSS.satellites(mySatellites))
    {

      Serial.print("SATELLITES: ");
      Serial.print(mySatellites.count());

      Serial.println();

      for (unsigned int index = 0; index < mySatellites.count(); index++)
      {
        unsigned int svid = mySatellites.svid(index);

        if ((svid >= 1) && (svid <= 32))
        {
          Serial.print("    ");

          if (svid <= 9)
          {
            Serial.print("  G");
            Serial.print(svid);
          }
          else
          {
            Serial.print(" G");
            Serial.print(svid);
          }
        }
        else if ((svid >= 65) && (svid <= 96))
        {
          Serial.print("    ");

          if ((svid - 64) <= 9)
          {
            Serial.print("  R");
            Serial.print(svid - 64);
          }
          else
          {
            Serial.print(" R");
            Serial.print(svid - 64);
          }
        }
        else if ((svid >= 120) && (svid <= 158))
        {
          Serial.print("    ");
          Serial.print("S");
          Serial.print(svid);
        }
        else if ((svid >= 173) && (svid <= 182))
        {
          Serial.print("    ");
          Serial.print("  I");
          Serial.print(svid - 172);
        }
        else if ((svid >= 193) && (svid <= 197))
        {
          Serial.print("    ");
          Serial.print("  Q");
          Serial.print(svid - 192);
        }
        else if ((svid >= 211) && (svid <= 246))
        {
          Serial.print("    ");

          if ((svid - 210) <= 9)
          {
            Serial.print("  E");
            Serial.print(svid - 210);
          }
          else
          {
            Serial.print(" E");
            Serial.print(svid - 210);
          }
        }
        else if (svid == 255)
        {
          Serial.print("    ");
          Serial.print("R???");
        }
        else
        {
          continue;
        }

        Serial.print(": SNR=");
        Serial.print(mySatellites.snr(index));
        Serial.print(", ELEVATION=");
        Serial.print(mySatellites.elevation(index));
        Serial.print(", AZIMUTH=");
        Serial.print(mySatellites.azimuth(index));

        if (mySatellites.unhealthy(index)) {
          Serial.print(", UNHEALTHY");
        }

        if (mySatellites.almanac(index)) {
          Serial.print(", ALMANAC");
        }

        if (mySatellites.ephemeris(index)) {
          Serial.print(", EPHEMERIS");
        }

        if (mySatellites.autonomous(index)) {
          Serial.print(", AUTONOMOUS");
        }

        if (mySatellites.correction(index)) {
          Serial.print(", CORRECTION");
        }

        if (mySatellites.acquired(index)) {
          Serial.print(", ACQUIRED");
        }

        if (mySatellites.locked(index)) {
          Serial.print(", LOCKED");
        }

        if (mySatellites.navigating(index)) {
          Serial.print(", NAVIGATING");
        }

        Serial.println();
      }

    } /* end of GNSS Satellites handling */

    // --- Information about satellites --- //
    // if (GNSS.satellites(mySatellites)){

    //     if( Enable_SerialPrint_Data_GPS == true ){
    //       Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;
    //       Nb_Satellite = mySatellites.count() ;
    //     }

    // } // if (GNSS.satellites(mySatellites))


  } // while( (millis() - now <= 60000)   )

  GNSS.suspend() ;
  new_pos_gps = true;


  Serial.println("First Fix GPS done.") ;

}

void read_update_GPS_data( bool enable_serialPrint_data_GPS ) {

  Serial.println("Read_Update_GPS_Data");


  //STM32L0.wakeup() ;

  delay(10);

  GNSS.resume();

  delay(10);

  while (GNSS.busy()) { }                                     // Wait for set to complete
  Serial.println("Resume done");

  EHPE = 99.99f ;

  int now = millis() ;

  // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

  while ( (millis() - now <= time_out_GPS)  ) {

    if (flag_sampling_during_GPS == true)
    {
      flag_sampling_during_GPS = false;
      Serial.println("Z, inc++ and X,Y during GPS search");
      inc++; // Increment number of dive sample

      // Sampling pressure
      // Pressure MS5837 Data
      MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
      MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
      MS5837_dT = MS5837_D2 - MS5837_Pcal[5] * pow(2, 8); // calculate temperature difference from reference
      MS5837_OFFSET = MS5837_Pcal[2] * pow(2, 16) + MS5837_dT * MS5837_Pcal[4] / pow(2, 7);
      MS5837_SENS = MS5837_Pcal[1] * pow(2, 15) + MS5837_dT * MS5837_Pcal[3] / pow(2, 8);

      MS5837_Temperature = (2000 + (MS5837_dT * MS5837_Pcal[6]) / pow(2, 23)) / 100;     // First-order Temperature in degrees Centigrade

      // Second order corrections
      if (MS5837_Temperature > 20)
      {
        MS5837_TT2 = 2 * MS5837_dT * MS5837_dT / pow(2, 37); // correction for high temperatures
        MS5837_OFFSET2 = 1 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 16;
        MS5837_SENS2 = 0;
      }
      if (MS5837_Temperature < 20)                  // correction for low temperature
      {
        MS5837_TT2      = 3 * MS5837_dT * MS5837_dT / pow(2, 33);
        MS5837_OFFSET2 = 3 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 2;
        MS5837_SENS2   = 5 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 8;
      }
      if (MS5837_Temperature < -15)                     // correction for very low temperature
      {
        MS5837_OFFSET2 = MS5837_OFFSET2 + 7 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
        MS5837_SENS2 = MS5837_SENS2 + 4 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
      }
      // End of second order corrections

      MS5837_Temperature = MS5837_Temperature - MS5837_TT2 / 100;
      MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
      MS5837_SENS = MS5837_SENS - MS5837_SENS2;

      MS5837_Pressure = (((MS5837_D1 * MS5837_SENS) / pow(2, 21) - MS5837_OFFSET) / pow(2, 13)) / 10; // Pressure in mbar or hPa

      // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
      float MS5837_depth = (MS5837_Pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);
      z[inc] = int16_t(MS5837_depth * 100) - offset_z;
      Serial.print("Depth :"); Serial.println(MS5837_depth);
      Serial.print("Depth_z :"); Serial.println(z[inc]);




    } // end of wake/sleep motion detection

    if (z[inc] < surf_depth) {
      Serial.print("end GPS, depth > threshold");
      break;
    }

    // Serial.println( (String)"While loop - condition n°1 : " + (millis() - now <= 30000) ) ;
    // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

    // Serial.println("Hello");



    // --- GNSS --- //
    //if (GNSS.location(myLocation) && GNSS.satellites(mySatellites) ) {
    if (GNSS.location(myLocation)  ) {

      if ( enable_serialPrint_data_GPS == true ) {
        Serial.print("LOCATION: ");
        Serial.print(fixTypeString[myLocation.fixType()]);
        if ( fixTypeString[myLocation.fixType()] == "NONE") {
          Serial.println(".") ;
        }
      }

      if (myLocation.fixType() != GNSSLocation::TYPE_NONE) {

        GPS_Year   = myLocation.year()    ;
        GPS_Month  = myLocation.month()   ;
        GPS_Day    = myLocation.day()     ;
        GPS_Hour   = myLocation.hours()   ;
        GPS_Minute = myLocation.minutes() ;
        GPS_Second = myLocation.seconds() ;

        // Display date
        if ( enable_serialPrint_data_GPS == true ) {
          Serial.print(fixQualityString[myLocation.fixQuality()]);
          Serial.print(" ");
          Serial.print(myLocation.year());
          Serial.print("/");
          Serial.print(myLocation.month());
          Serial.print("/");
          Serial.print(myLocation.day());
          Serial.print(" ");

          // Display hours
          if (myLocation.hours() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.hours());
          Serial.print(":");

          if (myLocation.minutes() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.minutes());
          Serial.print(":");

          if (myLocation.seconds() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.seconds());


          if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED) {
            Serial.print(" ");
            Serial.print(myLocation.leapSeconds());
            if (!myLocation.fullyResolved()) {
              Serial.print("D");
            }
          }

        }

        if (myLocation.fixType() != GNSSLocation::TYPE_TIME) {

          Lat = myLocation.latitude();
          myLocation.latitude(latOut);
          Long = myLocation.longitude();
          myLocation.longitude(longOut);
          Alt = myLocation.altitude();
          EHPE = myLocation.ehpe(); // use this as accuracy figure of merit
          Nb_Satellite = mySatellites.count() ;

          if ( enable_serialPrint_data_GPS == true ) {
            Serial.print(" : ");
            Serial.print(Lat, 7); Serial.print(","); Serial.print(Long, 7);
            Serial.print(",");
            Serial.print(Alt, 3);
            Serial.print(" EHPE :");
            Serial.print(EHPE, 3);
            Serial.print(",");
            Serial.print(myLocation.evpe(), 3);
            Serial.print(" SAT. :");
            Serial.print(myLocation.satellites());
            Serial.print(" DOP :");
            Serial.print(myLocation.hdop(), 2);
            Serial.print(",");
            Serial.print(myLocation.vdop(), 2);
            Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;

          }



          // if( (myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE < 10.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          // if( (EPE < 50.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          //   Serial.println("***GNSS go to sleep!***");
          //   GNSS.suspend(); // once we have a good 3D location fix put CAM M8Q to sleep
          //   //break ;
          // }

        } // if (myLocation.fixType() != GNSSLocation::TYPE_TIME)

      } // if (myLocation.fixType() != GNSSLocation::TYPE_NONE)

    } // if (GNSS.location(myLocation))

    if (GNSS.satellites(mySatellites))
    {

      Serial.print("SATELLITES: ");
      Serial.print(mySatellites.count());

      Serial.println();

      for (unsigned int index = 0; index < mySatellites.count(); index++)
      {
        unsigned int svid = mySatellites.svid(index);

        if ((svid >= 1) && (svid <= 32))
        {
          Serial.print("    ");

          if (svid <= 9)
          {
            Serial.print("  G");
            Serial.print(svid);
          }
          else
          {
            Serial.print(" G");
            Serial.print(svid);
          }
        }
        else if ((svid >= 65) && (svid <= 96))
        {
          Serial.print("    ");

          if ((svid - 64) <= 9)
          {
            Serial.print("  R");
            Serial.print(svid - 64);
          }
          else
          {
            Serial.print(" R");
            Serial.print(svid - 64);
          }
        }
        else if ((svid >= 120) && (svid <= 158))
        {
          Serial.print("    ");
          Serial.print("S");
          Serial.print(svid);
        }
        else if ((svid >= 173) && (svid <= 182))
        {
          Serial.print("    ");
          Serial.print("  I");
          Serial.print(svid - 172);
        }
        else if ((svid >= 193) && (svid <= 197))
        {
          Serial.print("    ");
          Serial.print("  Q");
          Serial.print(svid - 192);
        }
        else if ((svid >= 211) && (svid <= 246))
        {
          Serial.print("    ");

          if ((svid - 210) <= 9)
          {
            Serial.print("  E");
            Serial.print(svid - 210);
          }
          else
          {
            Serial.print(" E");
            Serial.print(svid - 210);
          }
        }
        else if (svid == 255)
        {
          Serial.print("    ");
          Serial.print("R???");
        }
        else
        {
          continue;
        }

        Serial.print(": SNR=");
        Serial.print(mySatellites.snr(index));
        Serial.print(", ELEVATION=");
        Serial.print(mySatellites.elevation(index));
        Serial.print(", AZIMUTH=");
        Serial.print(mySatellites.azimuth(index));

        if (mySatellites.unhealthy(index)) {
          Serial.print(", UNHEALTHY");
        }

        if (mySatellites.almanac(index)) {
          Serial.print(", ALMANAC");
        }

        if (mySatellites.ephemeris(index)) {
          Serial.print(", EPHEMERIS");
        }

        if (mySatellites.autonomous(index)) {
          Serial.print(", AUTONOMOUS");
        }

        if (mySatellites.correction(index)) {
          Serial.print(", CORRECTION");
        }

        if (mySatellites.acquired(index)) {
          Serial.print(", ACQUIRED");
        }

        if (mySatellites.locked(index)) {
          Serial.print(", LOCKED");
        }

        if (mySatellites.navigating(index)) {
          Serial.print(", NAVIGATING");
        }

        Serial.println();
      }

    } /* end of GNSS Satellites handling */

    // --- Information about satellites --- //
    // if (GNSS.satellites(mySatellites)){

    //     if( Enable_SerialPrint_Data_GPS == true ){
    //       Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;
    //       Nb_Satellite = mySatellites.count() ;
    //     }

    // } // if (GNSS.satellites(mySatellites))

    // --- Break Loop --- //

    if ( (EHPE < 10.0f)  ) { // 10 is about as low as one should go, 50 is acceptable
      Serial.println( (String)"Break at " + EHPE);
      new_pos_gps = true;
      break ;
    }

    delay(100);

  } // while( (millis() - now <= 60000)   )

  //samplingDuringGPS.stop();
  GNSS.suspend() ;

  //STM32L0.stop() ;

}


void read_battery_level( bool enable_serialPrint_battery ) {

  VDDA = STM32L0.getVDDA();                                               // Positive Voltage for Analog

  digitalWrite(myVBat_en, HIGH);                                           // Allow check Battery level
  battery_level = 1.27f * VDDA * analogRead(myVBat) / 4096.0f ;  // Determine Battery level
  digitalWrite(myVBat_en, LOW);                                            // Close pin

  if ( enable_serialPrint_battery == true ) {
    Serial.print("Battery Level: ");
    Serial.print(battery_level, 2);
    Serial.println(" V");
  }

}
























void printAODR()
{
  switch (AODR)
  {
    case 0x01:
      Serial.println("Data rate  = 1 Hz");
      break;
    case 0x02:
      Serial.println("Data rate  = 10 Hz");
      break;
    case 0x03:
      Serial.println("Data rate  = 25 Hz");
      break;
    case 0x04:
      Serial.println("Data rate  = 50 Hz");
      break;
    case 0x05:
      Serial.println("Data rate  = 100 Hz");
      break;
    default:
      Serial.println("Data rate Error");
      break;
  }
}


void printAscale()
{
  switch (Ascale)
  {
    case 0:
      Serial.println("Full scale = 2 g");
      break;
    case 1:
      Serial.println("Full scale = 4 g");
      break;
    case 2:
      Serial.println("Full scale = 8 g");
      break;
    case 3:
      Serial.println("Full scale = 16 g");
      break;
    default:
      Serial.println("Full scale Error");
      break;
  }
}


void getRTC (uint8_t index)
{
  RTC.getDate(day, month, year);
  RTC.getTime(hours[index], minutes[index], seconds[index], subSeconds[index]);
}

void printRTC (uint8_t index)
{
  //Serial.print(day); Serial.print(":"); Serial.print(month); Serial.print(":20"); Serial.print(year);
  //Serial.print(" ");

  milliseconds = ((subSeconds[index] >> 17) * 1000 + 16384) / 32768;

  if (hours[index] < 10)
  {
    Serial.print("0"); Serial.print(hours[index]);
  }
  else
    Serial.print(hours[index]);

  Serial.print(":");
  if (minutes[index] < 10)
  {
    Serial.print("0"); Serial.print(minutes[index]);
  }
  else
    Serial.print(minutes[index]);

  Serial.print(":");
  if (seconds[index] < 10)
  {
    Serial.print("0"); Serial.print(seconds[index]);
  }
  else
    Serial.print(seconds[index]);

  Serial.print(".");
  if (milliseconds <= 9)
  {
    Serial.print("0");
  }
  if (milliseconds <= 99)
  {
    Serial.print("0");
  }
  Serial.print(milliseconds);
  Serial.println(" ");
}


void SetDefaultRTC()   // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];    // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for (uint8_t i = 0; i < 3; i++)
  {
    build_mo += Build_mo[i];
  }
  if (build_mo == "Jan")
  {
    month = 1;
  } else if (build_mo == "Feb")
  {
    month = 2;
  } else if (build_mo == "Mar")
  {
    month = 3;
  } else if (build_mo == "Apr")
  {
    month = 4;
  } else if (build_mo == "May")
  {
    month = 5;
  } else if (build_mo == "Jun")
  {
    month = 6;
  } else if (build_mo == "Jul")
  {
    month = 7;
  } else if (build_mo == "Aug")
  {
    month = 8;
  } else if (build_mo == "Sep")
  {
    month = 9;
  } else if (build_mo == "Oct")
  {
    month = 10;
  } else if (build_mo == "Nov")
  {
    month = 11;
  } else if (build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if (build_date[4] != 32)                                                                           // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48) * 10 + build_date[5]  - 48;                                         // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48) * 10 + build_date[10] - 48;
  hours[0]   = (build_time[0] - 48) * 10 + build_time[1]  - 48;
  minutes[0] = (build_time[3] - 48) * 10 + build_time[4]  - 48;
  seconds[0] = (build_time[6] - 48) * 10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours[0]);
  RTC.setMinutes(minutes[0]);
  RTC.setSeconds(seconds[0]);
}












void fct_end_dive()
{
  if (start_dive_flag == 1 && z[inc] < 25) {
    etho[inc_eth][3] = inc;
    start_dive_flag = 0;
    end_dive_flag = 1;
  }
}


void etho_fct()
{
  int nb_sample_end;

  if (inc > 30)
  {
    start_dive_flag = 1;
  }


  if (inc_eth == 0) // Starting behavior of a dive
  {
    if (z[inc] < 1)
    {
      etho[inc_eth][1] = 3;  // Down phase
      etho[inc_eth][2] = 1;
      inc_eth++;
    } else {
      etho[inc_eth][1] = 5;  // sub_surface phase
      etho[inc_eth][2] = 1;
      inc_eth++;
    }
  }

  //Starting behavior
  //UP : Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope < slope_up)
  {
    flag_s[3][1]++;
    flag_s[3][2] = inc;

  } else {
    flag_s[3][1] = 0;
  }


  //DOWN :Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope > slope_down)
  {
    flag_s[4][1]++;
    flag_s[4][2] = inc;
  } else {
    flag_s[4][1] = 0;
  }

  //REST : Test if ODBA inferior to a threshold. We can slope also
  if (z[inc] < VeDBA_rest)
  {
    flag_s[2][1]++;
    flag_s[2][2] = inc;
  } else {
    flag_s[2][1] = 0;
  }

  //SURFACE : Test if depth is inferior to a threshold
  if (z[inc] < surf_depth)
  {
    flag_s[5][1]++;
    flag_s[5][2] = inc;
  } else {
    flag_s[5][1] = 0;
  }

  //Ending behavior
  //DOWN :Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope < slope_down  && etho[inc_eth][1] == 4)
  {
    flag_e[4][1]++;
    flag_e[4][2] = inc;
  } else {
    flag_e[4][1] = 0;
  }

  //UP : Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope < slope_down  && etho[inc_eth][1] == 3)
  {
    flag_e[3][1]++;
    flag_e[3][2] = inc;
  } else {
    flag_e[3][1] = 0;
  }

  //REST : Test if ODBA inferior to a threshold. We can slope also
  if (int16_t(VeDBA * 10000) > VeDBA_rest && etho[inc_eth][1] == 2)
  {
    flag_e[2][1]++;
    flag_e[2][2] = inc;
  } else {
    flag_e[2][1] = 0;
  }

  // Change behavior section

  //DOWN phase -> UP, REST
  if (etho[inc_eth][1] == 4)
  {
    if (flag_s[3][1] >= t_d2u) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[3][2] - flag_s[3][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 3;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    } else if ( flag_s[2][1] >= t_d2r)
    {
      nb_sample_end = flag_s[2][2] - flag_s[2][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 2;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));

    }
  }

  //Rest phase -> UP, DOWN
  if (etho[inc_eth][1] == 2)
  {
    if (flag_s[3][1] >= t_r2u) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[3][2] - flag_s[3][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 3;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    } else if ( flag_s[4][1] >= t_r2d)
    {
      nb_sample_end = flag_s[4][2] - flag_s[4][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 4;                 // Start DOWN behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));

    }
  }

  //UP phase -> SURFACE
  if (etho[inc_eth][1] == 3)
  {
    if (flag_s[5][1] >= t_u2s) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[5][2] - flag_s[5][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 5;                 // Start Surface behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }
  }

  //Surface phase -> DOWN (If the dive crop is well done nothing after surface)
  if (etho[inc_eth][1] == 5)
  {
    if (flag_s[4][1] >= t_s2d) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[4][2] - flag_s[4][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 4;                 // Start Down behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }
  }

  //SWIM phase -> DOWN, UP, REST
  if (etho[inc_eth][1] == 1)
  {
    if (flag_s[2][1] >= t_s2r) // REST: Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[2][2] - flag_s[2][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 2;                 // Start REST behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    } else if ( flag_s[3][1] >= t_s2u) //UP : compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[3][2] - flag_s[3][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 3;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));

    }
    else if ( flag_s[4][1] >= t_s2d) //DOWN : compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[4][2] - flag_s[4][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 4;                 // Start DOWN behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }
  }

  //SWIM : Swimming phase is hard to describe so we use the end time of the other phase
  //We use end flag of other behavior to go in SWIM phase
  if (flag_e[4][1] >= t_d2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[4][2] - flag_e[4][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  } else if (flag_e[5][1] >= t_surf2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[5][2] - flag_e[5][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  } else if (flag_e[3][1] >= t_u2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[3][2] - flag_e[3][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  } else if (flag_e[2][1] >= t_r2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[2][2] - flag_e[2][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  }


}

void speed_fct_1(int16_t behavior) {
  // speed calculation with fct 1

  if (behavior == 1) {
    h_speed = 0.83; //0.833; //0.69; //1.11; //0.69;
    //speed_h[inc] = 145;
  } else
  {
    h_speed = 0; //0;
    //speed_h[inc] = 0;
  }
  //int diff_d = abs(z[inc] - z[inc-1]);// * 1; // With fs = 1;
  //int v_speed = int(diff_d) / sin(pitch);
  /*}

    if (etho[1] = 2) {
    h_speed = 0.45;
    }

    if (etho[1] = 3) {
    h_speed = 0.45;
    }

    if (etho[1] = 4) {
    h_speed = 0.45;
    }*/

  //return h_speed;
}

float speed_fct_2(float f_depth, float f_last_depth, float pitch) {

  // speed calculation with fct 1
  float v_speed = 0;
  float diff_d;

  if (pitch * 100 > p_lim * 100)
  {
    diff_d = abs(f_last_depth - f_depth) * 1; // With fs = 1;
    v_speed = diff_d / sin(pitch);
  }

  h_speed = sqrt(max(pow(fixed_speed, 2) - pow(v_speed, 2), 0));
  return h_speed;
}



void lowpass_filter(float buf_acc_x[], float buf_acc_y, float buf_acc_z, float& acc_gx, float& acc_gy, float& acc_gz)
{

}


int32_t tr_area(int16_t X1, int16_t Y1, int16_t X_S, int16_t Y_S, int16_t X_E, int16_t Y_E)
{

  float area = 0.5 * ((float(X_S) - float(X1)) * (float(Y_E) - float(Y1)) - (float(X_E) - float(X1)) * (float(Y_S) - float(Y1)));

  Serial.print("Calculate Area 1: ");  Serial.println(area);

  if (area < 0)
  {
    area = -1 * area;
  }

  //Serial.print("area: ");  Serial.println(area);
  return int32_t(area);
}

void htrack_fct(int first_data_b, int last_data_b, bool debug)
{
  float x_buf;
  float y_buf;

  for (int i = first_data_b; i <= last_data_b; i++)
  {

    x_buf = /*float(x[INC - 1]) +*/ h_speed * cos(float(heading_array[i]) / 10000);
    y_buf = /*float(y[INC - 1]) +*/ h_speed * sin(float(heading_array[i]) / 10000);

    float d = sqrt(  pow(x_buf, 2) + pow( y_buf, 2));
    d_t = d_t + d; // Total distance travelled

    buf_1[i] = buf_1[i - 1] + int16_t(x_buf * 100);
    buf_2[i] = buf_2[i - 1] + int16_t(y_buf * 100);

    if (debug) {
      Serial.print(" yaw : ");  Serial.println(float(heading_array[i]) / 10000);
      Serial.print("sin yaw : ");  Serial.println(sin(float(heading_array[i]) / 10000));
      Serial.print("x_buf : ");  Serial.println(x_buf);
      Serial.print("y_buf : ");  Serial.println(y_buf);
      Serial.print("buf_1 : ");  Serial.print(buf_1[i]);
      Serial.print("buf_2 : ");  Serial.println(buf_2[i]);
    }
  }
}



void Visvalingam(float eps)
{


  //Serial.print("length of X1 : ");  Serial.println(len);
  uint32_t min_area = 32000;
  int ind = 0;
  int nb_pts = len;
  int flag_first_min = 0;
  //float X_out[nb_pts_max];
  //float Y_out[nb_pts_max];

  int i = 2;
  int i_b = 1;

  //Serial.print("len : ");  Serial.println(len);
  //Serial.print("nb_pts_max : ");  Serial.println(nb_pts_max);
  while (nb_pts > nb_pts_max)
    //for(int bb = 1; bb < 1549680;bb++)
  {

    //Serial.print("nb_pts : ");  Serial.println(nb_pts);

    // Calcul de l'aire de chaque triangle defini par 3 pts successifs
    for (int i_a = 3; i_a < len; i_a++)
    {

      //Serial.print("I_a : ");  Serial.println(i_a);

      while ( buf_1[i_a] == 32000 && i_a < len)
      {
        Serial.print("i_a ==32000 ");
        i_a++;
      }

      if (i_a == len)
      {
        //Serial.print("i_a == len ");
        break;
      }



      //uint32_t Area_size = tr_area(x[i_b], y[i_b], x[i], y[i], x[i_a], y[i_a]);
      int16_t area;
      area = /*0.5f **/ (((buf_1[i_b]) - (buf_1[i])) * ((buf_2[i_b]) - (buf_2[i])) - ((buf_1[i_b]) - (buf_1[i])) * ((buf_2[i_a]) - (buf_2[i])));
      //Serial.print("Calculate Area 1: ");  Serial.println(area);

      if (area < 0)
      {
        area = -1 * area;
      }
      Serial.print("Calculate Area 2: ");  Serial.println(area);


      // threshold on area size in order to accelerate algorithm on first iteration
      if (area < eps)
      {
        //Serial.print("inf eps: ");  Serial.println(eps);
        area = 0;
      }

      // Recherche du/des triangle(s) avec l'aire minimale
      if (area == min_area && flag_first_min == 1)
      {
        //min_area = Area_size;
        ind_min[ind] = i;
        ind++;
        //flag_first_min = 0;
        /*Serial.println("Adding a min : ");
          Serial.print("min_area : ");  Serial.println(min_area);
          Serial.print("ind++ : ");  Serial.println(ind);*/
      }

      // Recherche du/des triangle(s) avec l'aire minimale
      if (area < min_area)
      {
        flag_first_min = 1;
        min_area = area;
        ind = 1;
        ind_min[ind - 1] = i;
        /*Serial.println("New min : ");
          Serial.print("min_area : ");  Serial.println(min_area);
          Serial.print("ind = 1 : ");  Serial.println(ind);*/
      }

      i_b = i;
      i = i_a;

      //Serial.print("I_a : ");  Serial.println(i_a);

    }


    // Suppression des triangles a aire min (suppression du point median du triangle) et mise a jour des points restants
    for (int j = 1; j < ind + 1; j++)
    {
      Serial.print("ind point suppr: ");  Serial.println(ind_min[j - 1]);
      buf_1[ind_min[j - 1]] = 32000;
      buf_2[ind_min[j - 1]] = 32000;
    }


    nb_pts = nb_pts - ind;
    ind = 0;

    Serial.print("nb_pts : ");  Serial.println(nb_pts);

    i = 2;
    i_b = 1;
    min_area = 32000;
    flag_first_min = 1;


  }

  int jj = 0;
  for (int ii = 3; ii < len ; ii++)
  {

    //Serial.print("X1 at the end : ");  Serial.println(X1[ii]);
    if (buf_1[ii] != 32000)
    {
      //x_out[jj] = x[ii];
      //y_out[jj] = y[ii];
      //Serial.print("X out: ");  Serial.println(x_out[jj]);
      jj++;
    }
    buf_1[ii] = 0;
    buf_2[ii] = 0;
  }

}
