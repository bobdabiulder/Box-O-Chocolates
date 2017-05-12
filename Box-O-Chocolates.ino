#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "SparkFunMPL3115A2.h"
#include <SparkFunLSM9DS1.h>
#include <EEPROM.h>

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
#define DECLINATION -2.31 // Declination (degrees) in Boulder, CO.

const int SPI_CS_PIN = 10;
const int EEPROM_ADDR = 0;
const String DATA_FILE_NAME = "Data";

int testNum = 0;

//Create an instance of the object
MPL3115A2 myPressure;   //Create pressure sensor object
LSM9DS1 imu;  //Create imu object
File dataFile;

void setup()
{
  Wire.begin();        // Join i2c bus
  Serial.begin(9600);  // Start serial for output
  Serial.println("Starting");

  //Pressure
  myPressure.begin(); // Get sensor online
  
  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  //IMU
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
  }

  //SD CARD
  testNum = EEPROM.read(EEPROM_ADDR) + 1;   //Set the test number to the previous test number+1
  EEPROM.write(EEPROM_ADDR, testNum);   //Update the test number in the eeprom
  if (!SD.begin(SPI_CS_PIN)) {    //Check to see if the sd card reader is working
    Serial.println("SD Card Init Failed!");
  }
  dataFile = SD.open(DATA_FILE_NAME+testNum, FILE_WRITE);
  if (dataFile) {
    dataFile.print("Data File ");
    dataFile.println(testNum);
    dataFile.close();
  }
}

float sensorData[21]; //altitude (m), pressure (pa), temp (c), 
                      //gyro array, accel array, mag array
              
//gyro array: calculated gx, gy, gz, raw gx, gy, gz
//accel array: calculated ax, ay, az, raw ax, ay, az
//mag array: calculated mx, my, mz, raw mx, my, mz

void loop()
{
  sensorData[0] = myPressure.readAltitude();
  sensorData[1] = myPressure.readPressure();
  sensorData[2] = myPressure.readTemp();

  updateIMU();
  sensorData[3] = imu.gx;
  sensorData[4] = imu.gy;
  sensorData[5] = imu.gz;
  sensorData[6] = imu.calcGyro(imu.gx);
  sensorData[7] = imu.calcGyro(imu.gy);
  sensorData[8] = imu.calcGyro(imu.gz);

  sensorData[9] = imu.ax;
  sensorData[10] = imu.ay;
  sensorData[11] = imu.az;
  sensorData[12] = imu.calcAccel(imu.ax);
  sensorData[13] = imu.calcAccel(imu.ay);
  sensorData[14] = imu.calcAccel(imu.az);

  sensorData[15] = imu.mx;
  sensorData[16] = imu.my;
  sensorData[17] = imu.mz;
  sensorData[18] = imu.calcMag(imu.mx);
  sensorData[19] = imu.calcMag(imu.my);
  sensorData[20] = imu.calcMag(imu.mz);

  String data = millis() + "  [";
  for (int i=0; i < sizeof(sensorData)-1; i++) {
    data += (String)sensorData[i] + ", ";
  }
  data += (String)sensorData[sizeof(sensorData)];
  data += "]";
  Serial.println(data);
  //writeData();
  
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
//  printAttitude(imu.ax, imu.ay, imu.az, 
//                 -imu.my, -imu.mx, imu.mz);

  //  float altitude = myPressure.readAltitudeFt();
//  Serial.print(" Altitude(ft):");
//  Serial.print(altitude, 2);

  //float pressure = myPressure.readPressure();
  //Serial.print("Pressure(Pa):");
  //Serial.print(pressure, 2);

  //float temperature = myPressure.readTemp();
  //Serial.print(" Temp(c):");
  //Serial.print(temperature, 2);

//  float temperature = myPressure.readTempF();
//  Serial.print(" Temp(f):");
//  Serial.print(temperature, 2);
}

void writeData(String data) {
  dataFile = SD.open(DATA_FILE_NAME+testNum, FILE_WRITE);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
  }
  else {
    Serial.print("Data write failed for '");
    Serial.print(data);
    Serial.println("'");
  }
}

void updateIMU() {
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the readGyro() function. When it exits, it'll update the gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the readAccel() function. When it exits, it'll update the ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the readMag() function. When it exits, it'll update the mx, my, and mz variables with the most current data.
    imu.readMag();
  }
}





//void printGyro() {
//  // Now we can use the gx, gy, and gz variables as we please.
//  // Either print them as raw ADC values, or calculated in DPS.
//  Serial.print("G: ");
//#ifdef PRINT_CALCULATED
//  // If you want to print calculated values, you can use the
//  // calcGyro helper function to convert a raw ADC value to
//  // DPS. Give the function the value that you want to convert.
//  Serial.print(imu.calcGyro(imu.gx), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcGyro(imu.gy), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcGyro(imu.gz), 2);
//  Serial.println(" deg/s");
//#elif defined PRINT_RAW
//  Serial.print(imu.gx);
//  Serial.print(", ");
//  Serial.print(imu.gy);
//  Serial.print(", ");
//  Serial.println(imu.gz);
//#endif
//}
//
//void printAccel()
//{  
//  // Now we can use the ax, ay, and az variables as we please.
//  // Either print them as raw ADC values, or calculated in g's.
//  Serial.print("A: ");
//#ifdef PRINT_CALCULATED
//  // If you want to print calculated values, you can use the
//  // calcAccel helper function to convert a raw ADC value to
//  // g's. Give the function the value that you want to convert.
//  Serial.print(imu.calcAccel(imu.ax), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcAccel(imu.ay), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcAccel(imu.az), 2);
//  Serial.println(" g");
//#elif defined PRINT_RAW 
//  Serial.print(imu.ax);
//  Serial.print(", ");
//  Serial.print(imu.ay);
//  Serial.print(", ");
//  Serial.println(imu.az);
//#endif
//
//}
//
//void printMag()
//{  
//  // Now we can use the mx, my, and mz variables as we please.
//  // Either print them as raw ADC values, or calculated in Gauss.
//  Serial.print("M: ");
//#ifdef PRINT_CALCULATED
//  // If you want to print calculated values, you can use the
//  // calcMag helper function to convert a raw ADC value to
//  // Gauss. Give the function the value that you want to convert.
//  Serial.print(imu.calcMag(imu.mx), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcMag(imu.my), 2);
//  Serial.print(", ");
//  Serial.print(imu.calcMag(imu.mz), 2);
//  Serial.println(" gauss");
//#elif defined PRINT_RAW
//  Serial.print(imu.mx);
//  Serial.print(", ");
//  Serial.print(imu.my);
//  Serial.print(", ");
//  Serial.println(imu.mz);
//#endif
//}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
//void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
//{
//  float roll = atan2(ay, az);
//  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
//  
//  float heading;
//  if (my == 0)
//    heading = (mx < 0) ? PI : 0;
//  else
//    heading = atan2(mx, my);
//    
//  heading -= DECLINATION * PI / 180;
//  
//  if (heading > PI) heading -= (2 * PI);
//  else if (heading < -PI) heading += (2 * PI);
//  else if (heading < 0) heading += 2 * PI;
//  
//  // Convert everything from radians to degrees:
//  heading *= 180.0 / PI;
//  pitch *= 180.0 / PI;
//  roll  *= 180.0 / PI;
//  
//  Serial.print("Pitch, Roll: ");
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.println(roll, 2);
//  Serial.print("Heading: "); Serial.println(heading, 2);
//}
