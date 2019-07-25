/* BMI160 Basic Example Code
 by: Afantor
 date:2019/06/02
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 This sketch uses SDA/SCL on pins 26/25, respectively.
 
 SDA and SCL has external pull-up resistors (to 3.3V).
 
 Hardware setup:
 BOARD ------------------- Bluefruit52 V3.0
 VDD ---------------------- 3.3V
 SDA ----------------------- 25
 SCL ----------------------- 26
 GND ---------------------- GND
 
 */
#include <bluefruit52.h>

const int irq_pin = 30;
const int i2c_addr = 0x69;

#define BMI160_ADDRESS        0x69  // Device address when ADO = 1
#define BMI160_STATUS         0x1B

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
#define  AFS_2G 0x03
#define  AFS_4G 0x05
#define  AFS_8G 0x08
#define  AFS_16G 0x0C

enum AODR {
  Arate0 = 0,
  Arate1Hz,  // really 25/32
  Arate2Hz,  // really 25/16
  Arate3Hz, // really 25/8
  Arate6_25Hz,  
  Arate12_5Hz,
  Arate25Hz,
  Arate50Hz,
  Arate100Hz,
  Arate200Hz,
  Arate400Hz,
  Arate800Hz,
  Arate1600Hz
};

enum ABW {
  ABW_4X = 0, // 4 times oversampling ~ 10% of ODR
  ABW_2X,     // 2 times oversampling ~ 20% of ODR
  ABW_1X      // 1 times oversampling ~ 40% of ODR
};

enum Gscale {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODR {
  Grate25Hz = 6,
  Grate50Hz,
  Grate100Hz,
  Grate200Hz,
  Grate400Hz,
  Grate800Hz,
  Grate1600Hz,
  Grate3200Hz
};

enum GBW {
  GBW_4X = 0, // 4 times oversampling ~ 10% of ODR
  GBW_2X,     // 2 times oversampling ~ 20% of ODR
  GBW_1X      // 1 times oversampling ~ 40% of ODR
};


//
// Specify sensor full scale
uint8_t Gscale = GFS_2000DPS, GODR = Grate3200Hz, GBW = GBW_2X;
uint8_t Ascale = AFS_2G, AODR = Arate1600Hz, ABW = ABW_2X;
float aRes, gRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int myLed     = 19;  // LED on the Bluefruit52

// BMI160 variables
int accelx,accely,accelz;  // Stores the 16-bit signed accelerometer sensor output
int gyrox,gyroy,gyroz;   // Stores the 16-bit signed gyro sensor output
float Quat[4] = {0, 0, 0, 0}; // quaternion data register
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, mag
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the BMX055 internal chip temperature in degrees Celsius
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll, Yaw, Pitch, Roll;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval
uint8_t param[4];                         // used for param transfer

float ax, ay, az, gx, gy, gz;             // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

bool passThru = true;

int calibrateOffsets = 1; // int to determine whether calibration takes place or not

void setup()
{
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin();
  delay(50);
  Serial.begin(115200);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("BMI160 6-axis motion sensor...");
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr, irq_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("BMI160,I AM 0x"); 
  Serial.println(dev_id, HEX); 
  Serial.print(" I should be "); Serial.println(0xD1, HEX);
   
  delay(1000); 

  if (dev_id == 0xD1) // WHO_AM_I should always be ACC/GYRO = 0xD1, MAG = 0x48 
  {  
    Serial.println("BMI160 are online...");
  
    delay(100); 
    // get sensor resolutions, only need to do this once
    getAres();
    getGres();
    // Set the gyro range to 2000 degrees/second
    BMI160.setGyroRate(3200);
    BMI160.setGyroRange(2000);
     // Set the accelerometer range to 2g
    BMI160.setAccelerometerRate(1600);
    BMI160.setAccelerometerRange(2);
  }
  else
  {
    Serial.print("Could not connect to BMI160: 0x");
    Serial.println(dev_id, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  // use the code below to calibrate accel/gyro offset values
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(BMI160.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(BMI160.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(BMI160.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //BMI160.setAccelerometerOffset(X_AXIS,495.3);
    //BMI160.setAccelerometerOffset(Y_AXIS,-15.6);
    //BMI160.setAccelerometerOffset(Z_AXIS,491.4);
    //BMI160.setGyroOffset(X_AXIS,7.869);
    //BMI160.setGyroOffset(Y_AXIS,-0.061);
    //BMI160.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    BMI160.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(BMI160.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(BMI160.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(BMI160.getGyroOffset(Z_AXIS));
  }
}


void loop()
{  
  if (readByte(BMI160_ADDRESS, BMI160_STATUS) & 0x40) {  // check if new gyro data
    // read raw accel/gyro measurements from device
    BMI160.readMotionSensor(accelx,accely,accelz, gyrox,gyroy,gyroz);
  
    // these methods (and a few others) are also available
  
    //BMI160.readAcceleration(ax, ay, az);
    //BMI160.readRotation(gx, gy, gz);
  
    //ax = BMI160.readAccelerometer(X_AXIS);
    //ay = BMI160.readAccelerometer(Y_AXIS);
    //az = BMI160.readAccelerometer(Z_AXIS);
    //gx = BMI160.readGyro(X_AXIS);
    //gy = BMI160.readGyro(Y_AXIS);
    //gz = BMI160.readGyro(Z_AXIS);

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelx*aRes;  // get actual g value, this depends on scale being set
    ay = (float)accely*aRes;  
    az = (float)accelz*aRes;  

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyrox*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroy*gRes;  
    gz = (float)gyroz*gRes;   
  }


 // keep track of rates
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Use NED orientaton convention where N is direction of arrow on the board
  // AK8963C has N = -y, E = -x, and D = -z if top of board == North
  // BMI160 has N = x, E - -y, and D - -z for the gyro, and opposite by convention for the accel 
  // This rotation can be modified to allow any convenient orientation convention.
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, deltat);
//  if(passThru)MahonyQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  -my,  -mx, -mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 500)  // update LCD once per half-second independent of read rate
  {
    // if(SerialDebug) {
    //   Serial.print("ax = "); Serial.print((int)1000*ax);  
    //   Serial.print(" ay = "); Serial.print((int)1000*ay); 
    //   Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    //   Serial.print("gx = "); Serial.print( gx, 2); 
    //   Serial.print(" gy = "); Serial.print( gy, 2); 
    //   Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      
    //   Serial.println("Software quaternions:"); 
    //   Serial.print("q0 = "); Serial.print(q[0]);
    //   Serial.print(" qx = "); Serial.print(q[1]); 
    //   Serial.print(" qy = "); Serial.print(q[2]); 
    //   Serial.print(" qz = "); Serial.println(q[3]); 
    // }               

   //  tempCount = readTempData();  // Read the gyro adc values
   //  temperature = ((float) tempCount) / 512.0 + 23.0; // Gyro chip temperature in degrees Centigrade
   // // Print temperature in degrees Centigrade      
   //  Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
   
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
    if(yaw < 0) yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    yaw   += 0.02f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    
    //Hardware AHRS:
    // Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);   
    // Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
    // Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
    // Pitch *= 180.0f / PI;
    // Yaw   *= 180.0f / PI; 
    // Yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    // if(Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
    // Roll  *= 180.0f / PI;
    
    // Or define output variable according to the Android system, where heading (0 to 260) is defined by the angle between the y-axis 
    // and True North, pitch is rotation about the x-axis (-180 to +180), and roll is rotation about the y-axis (-90 to +90)
    // In this systen, the z-axis is pointing away from Earth, the +y-axis is at the "top" of the device (cellphone) and the +x-axis
    // points toward the right of the device.
    //
    
    if(SerialDebug) {
      // Serial.print("Software Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);
      
      // Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }
 
    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float uint32_reg_to_float (uint8_t *buf)
{
  union {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 =     (((uint32_t)buf[0]) +
               (((uint32_t)buf[1]) <<  8) +
               (((uint32_t)buf[2]) << 16) +
               (((uint32_t)buf[3]) << 24));
  return u.f;
}

void float_to_bytes (float param_val, uint8_t *buf) {
  union {
    float f;
    uint8_t comp[sizeof(float)];
  } u;
  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = u.comp[i];
  }
  //Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++) {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_125DPS:
          gRes = 125.0/32768.0;
          break;
     case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (05), 8 Gs (08), and 16 Gs  (0C). 
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the MPU6500 and AK8963 sensors
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (uint8_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
	while (Wire.available()) {
    dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

